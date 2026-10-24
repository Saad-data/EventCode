#include "code128.hpp"

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_ll_biases.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    bool self_test = false;
    bool headless = false;
    bool stimulus_only = false;
    bool rgb_enabled = true;
    bool data_rate_plot = false;
    std::string input;
    std::string output_dir = "artifacts";
    int duration_seconds = 0;
    int stimulus_x = 3840;
    int stimulus_y = 0;
    int stimulus_width = 1920;
    int stimulus_height = 1080;
    int module_pixels = 6;
    int persistence_us = 30000;
    int decode_window_us = 12000;
    int rgb_device = -1;
};

void print_help(const char *program) {
    std::cout
        << "SilkyCam VGA event-based Code 128 demonstration\n\n"
        << "Usage: " << program << " [options]\n"
        << "  --self-test              Run deterministic Code 128 tests\n"
        << "  --headless               Capture/process without GUI stimulus\n"
        << "  --stimulus-only          Show moving barcodes without opening a camera\n"
        << "  --no-rgb                 Disable the RealSense comparison\n"
        << "  --rgb-device N           Override automatic RealSense color-node detection\n"
        << "  --data-rate-plot         Show live Event-ROI versus RGB data rates\n"
        << "  --no-data-rate-plot      Hide the live data-rate plot (default)\n"
        << "  --input FILE             Replay a Metavision RAW file\n"
        << "  --duration SECONDS       Stop automatically (0 means continuous)\n"
        << "  --output DIR             Diagnostics/log directory (default artifacts)\n"
        << "  --stimulus-x PIXELS      External display X origin (default 3840)\n"
        << "  --stimulus-size WxH      Render size (default 1920x1080)\n"
        << "  --module-pixels N        Barcode module size (default 6)\n"
        << "  --help                   Show this help\n\n"
        << "Keys: Q/Esc quit, Space toggle auto speed, 1-5 select speed, S save snapshot.\n";
}

Options parse_options(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char *name) -> std::string {
            if (++i >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[i];
        };
        if (arg == "--self-test") options.self_test = true;
        else if (arg == "--headless") options.headless = true;
        else if (arg == "--stimulus-only") options.stimulus_only = true;
        else if (arg == "--no-rgb") options.rgb_enabled = false;
        else if (arg == "--rgb-device") options.rgb_device = std::stoi(value("--rgb-device"));
        else if (arg == "--data-rate-plot") options.data_rate_plot = true;
        else if (arg == "--no-data-rate-plot") options.data_rate_plot = false;
        else if (arg == "--input") options.input = value("--input");
        else if (arg == "--duration") options.duration_seconds = std::stoi(value("--duration"));
        else if (arg == "--output") options.output_dir = value("--output");
        else if (arg == "--stimulus-x") options.stimulus_x = std::stoi(value("--stimulus-x"));
        else if (arg == "--module-pixels") options.module_pixels = std::stoi(value("--module-pixels"));
        else if (arg == "--stimulus-size") {
            const auto size = value("--stimulus-size");
            const auto split = size.find('x');
            if (split == std::string::npos) throw std::runtime_error("--stimulus-size must be WxH");
            options.stimulus_width = std::stoi(size.substr(0, split));
            options.stimulus_height = std::stoi(size.substr(split + 1));
        } else if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    if (options.module_pixels < 2 || options.module_pixels > 30) throw std::runtime_error("invalid module size");
    return options;
}

class EventSurface {
public:
    EventSurface(int width, int height) : width_(width), height_(height), stamp_(width * height, 0),
                                          raw_stamp_(width * height, 0), polarity_(width * height, 0) {}

    void ingest(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        if (begin == end) return;
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto event = begin; event != end; ++event) {
            if (event->x >= width_ || event->y >= height_) continue;
            const std::size_t index = static_cast<std::size_t>(event->y) * width_ + event->x;
            ++total_events_;

            // Background-activity filter: real barcode edges arrive in compact
            // space/time clusters. Isolated hot pixels do not have a recent
            // neighbour and are discarded. A short refractory period also
            // removes repeated firing from the same LCD transition.
            bool has_recent_neighbour = false;
            const Metavision::timestamp neighbour_after = event->t - 2500;
            for (int dy = -1; dy <= 1 && !has_recent_neighbour; ++dy) {
                const int yy = static_cast<int>(event->y) + dy;
                if (yy < 0 || yy >= height_) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int xx = static_cast<int>(event->x) + dx;
                    if (xx < 0 || xx >= width_) continue;
                    const auto neighbour = static_cast<std::size_t>(yy) * width_ + xx;
                    if (raw_stamp_[neighbour] >= neighbour_after && raw_stamp_[neighbour] <= event->t) {
                        has_recent_neighbour = true;
                        break;
                    }
                }
            }
            raw_stamp_[index] = event->t;
            if (!has_recent_neighbour) continue;
            if (stamp_[index] != 0 && event->t - stamp_[index] < 180) continue;
            stamp_[index] = event->t;
            polarity_[index] = event->p ? 1 : -1;
            ++accepted_events_;
            if (roi_enabled_ && event->x >= roi_left_ && event->x < roi_right_ &&
                event->y >= roi_top_ && event->y < roi_bottom_) {
                ++roi_events_;
            }
        }
        latest_timestamp_ = (end - 1)->t;
    }

    struct Snapshot {
        std::vector<Metavision::timestamp> stamp;
        std::vector<std::int8_t> polarity;
        Metavision::timestamp latest = 0;
        std::uint64_t total_events = 0;
        std::uint64_t accepted_events = 0;
        std::uint64_t roi_events = 0;
    };

    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {stamp_, polarity_, latest_timestamp_, total_events_, accepted_events_, roi_events_};
    }

    void set_roi(int left, int top, int right, int bottom) {
        std::lock_guard<std::mutex> lock(mutex_);
        roi_left_ = std::clamp(left, 0, width_);
        roi_right_ = std::clamp(right, 0, width_);
        roi_top_ = std::clamp(top, 0, height_);
        roi_bottom_ = std::clamp(bottom, 0, height_);
        roi_enabled_ = roi_right_ > roi_left_ && roi_bottom_ > roi_top_;
    }

    void clear_roi() {
        std::lock_guard<std::mutex> lock(mutex_);
        roi_enabled_ = false;
    }

private:
    int width_;
    int height_;
    mutable std::mutex mutex_;
    std::vector<Metavision::timestamp> stamp_;
    std::vector<Metavision::timestamp> raw_stamp_;
    std::vector<std::int8_t> polarity_;
    Metavision::timestamp latest_timestamp_ = 0;
    std::uint64_t total_events_ = 0;
    std::uint64_t accepted_events_ = 0;
    std::uint64_t roi_events_ = 0;
    bool roi_enabled_ = false;
    int roi_left_ = 0;
    int roi_right_ = 0;
    int roi_top_ = 0;
    int roi_bottom_ = 0;
};

class RgbCapture {
public:
    struct Snapshot {
        cv::Mat frame;
        std::string decoded;
        std::uint64_t sequence = 0;
        Clock::time_point captured_at{};
    };

    explicit RgbCapture(int device_index) : device_index_(device_index) {
        if (!capture_.open(device_index, cv::CAP_V4L2)) {
            throw std::runtime_error("could not open RealSense RGB /dev/video" + std::to_string(device_index));
        }
        const int requested_fourcc = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
        capture_.set(cv::CAP_PROP_FOURCC, requested_fourcc);
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture_.set(cv::CAP_PROP_FPS, 60);
        capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        capture_.set(cv::CAP_PROP_CONVERT_RGB, 1);

        // Give the RGB baseline its best reasonable motion performance: 60 fps
        // and short manual exposure. Unsupported controls are harmlessly ignored
        // by the V4L2 backend and the actual values are reported below.
        capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
        capture_.set(cv::CAP_PROP_EXPOSURE, 50.0);
        capture_.set(cv::CAP_PROP_GAIN, 32.0);
        const int actual_fourcc = static_cast<int>(std::lround(capture_.get(cv::CAP_PROP_FOURCC)));
        if (actual_fourcc != requested_fourcc || capture_.get(cv::CAP_PROP_FRAME_WIDTH) < 600 ||
            capture_.get(cv::CAP_PROP_FRAME_HEIGHT) < 460 || capture_.get(cv::CAP_PROP_FPS) < 50) {
            capture_.release();
            throw std::runtime_error("/dev/video" + std::to_string(device_index) +
                                     " is not the RealSense YUYV 640x480 60-fps color stream");
        }
        running_ = true;
        worker_ = std::thread([this] { capture_loop(); });
    }

    ~RgbCapture() { stop(); }
    RgbCapture(const RgbCapture &) = delete;
    RgbCapture &operator=(const RgbCapture &) = delete;

    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {latest_.clone(), decoded_, sequence_, captured_at_};
    }

    double fps() const { return capture_.get(cv::CAP_PROP_FPS); }
    double exposure() const { return capture_.get(cv::CAP_PROP_EXPOSURE); }
    int device_index() const { return device_index_; }

    void stop() {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) worker_.join();
        capture_.release();
    }

private:
    void capture_loop() {
        cv::Mat frame;
        while (running_) {
            if (!capture_.read(frame) || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const auto decode_result = barcode::decode_grayscale_frame(frame, 7);
            std::lock_guard<std::mutex> lock(mutex_);
            frame.copyTo(latest_);
            decoded_ = decode_result.valid ? decode_result.text : std::string();
            captured_at_ = Clock::now();
            ++sequence_;
        }
    }

    cv::VideoCapture capture_;
    int device_index_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex mutex_;
    cv::Mat latest_;
    std::string decoded_;
    std::uint64_t sequence_ = 0;
    Clock::time_point captured_at_{};
};

bool supports_yuyv_capture(const std::filesystem::path &device) {
    const int descriptor = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (descriptor < 0) return false;

    bool supported = false;
    v4l2_fmtdesc format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (::ioctl(descriptor, VIDIOC_ENUM_FMT, &format) == 0) {
        if (format.pixelformat == V4L2_PIX_FMT_YUYV) {
            supported = true;
            break;
        }
        ++format.index;
    }
    ::close(descriptor);
    return supported;
}

std::vector<int> find_realsense_video_nodes() {
    std::vector<int> nodes;
    const std::filesystem::path video_class("/sys/class/video4linux");
    std::error_code error;
    if (!std::filesystem::exists(video_class, error)) return nodes;

    for (const auto &entry : std::filesystem::directory_iterator(video_class, error)) {
        const std::string filename = entry.path().filename().string();
        if (filename.rfind("video", 0) != 0) continue;
        std::ifstream name_file(entry.path() / "name");
        std::string device_name;
        std::getline(name_file, device_name);
        if (device_name.find("RealSense") == std::string::npos) continue;
        try {
            const int node = std::stoi(filename.substr(5));
            if (supports_yuyv_capture("/dev/video" + std::to_string(node))) nodes.push_back(node);
        } catch (const std::exception &) {
        }
    }
    std::sort(nodes.begin(), nodes.end());
    return nodes;
}

std::unique_ptr<RgbCapture> open_realsense_rgb(int requested_device) {
    if (requested_device >= 0) return std::make_unique<RgbCapture>(requested_device);

    const auto candidates = find_realsense_video_nodes();
    if (candidates.empty()) {
        throw std::runtime_error("no Intel RealSense video devices were found");
    }

    std::string errors;
    for (const int candidate : candidates) {
        try {
            return std::make_unique<RgbCapture>(candidate);
        } catch (const std::exception &exception) {
            if (!errors.empty()) errors += "; ";
            errors += "/dev/video" + std::to_string(candidate) + ": " + exception.what();
        }
    }
    throw std::runtime_error("RealSense is connected, but its RGB stream could not be selected (" + errors + ")");
}

struct Candidate {
    barcode::EncodedCode128 code;
    cv::Mat image;
};

struct Match {
    bool valid = false;
    std::string text;
    double score = 0.0;
    int left = 0;
    int right = 0;
    double module_scale = 0.0;
    double margin = 0.0;
};

double sample_linear(const std::vector<float> &values, double x) {
    if (x < 0.0 || x > static_cast<double>(values.size() - 1)) return 0.0;
    const int x0 = static_cast<int>(x);
    const int x1 = std::min<int>(x0 + 1, static_cast<int>(values.size() - 1));
    const double fraction = x - x0;
    return values[x0] * (1.0 - fraction) + values[x1] * fraction;
}

Match match_known_codes(const std::vector<float> &profile, const std::vector<float> &on_profile,
                        const std::vector<float> &off_profile, const std::vector<Candidate> &candidates,
                        double calibrated_scale) {
    Match best;
    if (profile.empty()) return best;
    const float peak = *std::max_element(profile.begin(), profile.end());
    if (peak < 8.0f) return best;
    double second_candidate_score = -1e9;

    for (const auto &candidate : candidates) {
        double candidate_best = -1e9;
        Match candidate_match;
        const auto &edges = candidate.code.edge_modules;
        const double scale_min = calibrated_scale > 0.0 ? std::max(1.0, calibrated_scale * 0.55) : 1.0;
        const double scale_max = calibrated_scale > 0.0 ? std::min(5.5, calibrated_scale * 1.45) : 5.5;
        for (double scale = scale_min; scale <= scale_max; scale += 0.08) {
            const int barcode_width = static_cast<int>(std::lround(candidate.code.modules * scale));
            if (barcode_width >= static_cast<int>(profile.size()) - 4) continue;
            for (int left = 0; left + barcode_width < static_cast<int>(profile.size()); left += 2) {
                double best_phase_sum = -1e9;
                double absolute_edge_sum = 0.0;
                int edge_hits = 0;
                for (const int module : edges) absolute_edge_sum += sample_linear(profile, left + module * scale);
                for (int phase = 0; phase < 2; ++phase) {
                    double phase_sum = 0.0;
                    int phase_hits = 0;
                    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
                        const double x = left + edges[edge_index] * scale;
                        const bool expected_on = ((static_cast<int>(edge_index) + phase) % 2) == 0;
                        const double correct = sample_linear(expected_on ? on_profile : off_profile, x);
                        const double wrong = sample_linear(expected_on ? off_profile : on_profile, x);
                        phase_sum += correct - 0.55 * wrong;
                        if (correct > peak * 0.18) ++phase_hits;
                    }
                    if (phase_sum > best_phase_sum) {
                        best_phase_sum = phase_sum;
                        edge_hits = phase_hits;
                    }
                }
                const double polarity_mean = best_phase_sum / edges.size();
                const double absolute_edge_mean = absolute_edge_sum / edges.size();
                double middle_sum = 0.0;
                int run_start = 0;
                for (const int run : candidate.code.runs) {
                    middle_sum += sample_linear(profile, left + (run_start + run * 0.5) * scale);
                    run_start += run;
                }
                const double middle_mean = middle_sum / candidate.code.runs.size();
                const double coverage = static_cast<double>(edge_hits) / edges.size();
                const double score = (0.68 * absolute_edge_mean + 0.32 * polarity_mean - 0.22 * middle_mean) /
                                     (peak + 1e-6) * (0.35 + 0.65 * coverage);
                if (score > candidate_best) {
                    candidate_best = score;
                    candidate_match = {false, candidate.code.text, score, left, left + barcode_width, scale, 0.0};
                }
            }
        }
        if (candidate_best > best.score) {
            second_candidate_score = best.score;
            best = candidate_match;
        } else if (candidate_best > second_candidate_score) {
            second_candidate_score = candidate_best;
        }
    }
    best.margin = best.score - second_candidate_score;
    // Ambiguous single frames are allowed into temporal consensus; they are
    // never reported directly. Low-margin matches must persist longer below.
    best.valid = best.score > 0.24;
    return best;
}

std::vector<double> extract_edges(const std::vector<float> &profile, int left, int right) {
    std::vector<double> edges;
    if (profile.empty() || right <= left) return edges;
    const float peak = *std::max_element(profile.begin() + left, profile.begin() + right + 1);
    const float threshold = peak * 0.32f;
    bool inside = false;
    double weighted_x = 0.0;
    double weight = 0.0;
    for (int x = left; x <= right; ++x) {
        if (profile[x] >= threshold) {
            inside = true;
            weighted_x += x * profile[x];
            weight += profile[x];
        } else if (inside) {
            if (weight > 0.0) edges.push_back(weighted_x / weight);
            inside = false;
            weighted_x = weight = 0.0;
        }
    }
    if (inside && weight > 0.0) edges.push_back(weighted_x / weight);
    return edges;
}

struct ProcessedFrame {
    cv::Mat image;
    std::vector<float> profile;
    std::vector<float> on_profile;
    std::vector<float> off_profile;
    int roi_top = 0;
    int roi_bottom = 0;
    std::size_t active_pixels = 0;
};

ProcessedFrame process_snapshot(const EventSurface::Snapshot &snapshot, int width, int height,
                                int persistence_us, int decode_window_us) {
    ProcessedFrame output;
    output.image = cv::Mat(height, width, CV_8UC3, cv::Scalar(24, 24, 24));
    std::vector<int> row_activity(height, 0);
    const auto visible_after = snapshot.latest - persistence_us;
    const auto decode_after = snapshot.latest - decode_window_us;

    for (int y = 0; y < height; ++y) {
        auto *pixels = output.image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width + x;
            if (snapshot.stamp[index] >= visible_after) {
                pixels[x] = snapshot.polarity[index] > 0 ? cv::Vec3b(60, 220, 255) : cv::Vec3b(255, 90, 80);
                ++output.active_pixels;
            }
            if (snapshot.stamp[index] >= decode_after) ++row_activity[y];
        }
    }

    // A narrow scan band prevents perspective and LCD scan-out skew from
    // smearing the nominally vertical barcode edges in the X projection.
    const int band_height = std::max(32, static_cast<int>(height * 0.10));
    int running = std::accumulate(row_activity.begin(), row_activity.begin() + band_height, 0);
    int best_sum = running;
    int best_top = 0;
    for (int top = 1; top + band_height <= height; ++top) {
        running += row_activity[top + band_height - 1] - row_activity[top - 1];
        if (running > best_sum) { best_sum = running; best_top = top; }
    }
    output.roi_top = best_top;
    output.roi_bottom = std::min(height - 1, best_top + band_height - 1);
    output.profile.assign(width, 0.0f);
    output.on_profile.assign(width, 0.0f);
    output.off_profile.assign(width, 0.0f);
    for (int y = output.roi_top; y <= output.roi_bottom; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width + x;
            if (snapshot.stamp[index] >= decode_after) {
                output.profile[x] += 1.0f;
                (snapshot.polarity[index] > 0 ? output.on_profile[x] : output.off_profile[x]) += 1.0f;
            }
        }
    }
    std::vector<float> smooth(width, 0.0f);
    std::vector<float> smooth_on(width, 0.0f);
    std::vector<float> smooth_off(width, 0.0f);
    for (int x = 0; x < width; ++x) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int xx = std::clamp(x + dx, 0, width - 1);
            const int weight = 3 - std::abs(dx);
            smooth[x] += output.profile[xx] * weight;
            smooth_on[x] += output.on_profile[xx] * weight;
            smooth_off[x] += output.off_profile[xx] * weight;
        }
        smooth[x] /= 9.0f;
        smooth_on[x] /= 9.0f;
        smooth_off[x] /= 9.0f;
    }
    output.profile.swap(smooth);
    output.on_profile.swap(smooth_on);
    output.off_profile.swap(smooth_off);
    return output;
}

void put_label(cv::Mat &image, const std::string &text, cv::Point origin, double scale,
               cv::Scalar color, int thickness = 2) {
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness + 3, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

cv::Mat render_data_rate_plot(int width, int height, const std::deque<double> &event_history,
                              const std::deque<double> &rgb_history, double event_mb_s, double rgb_mb_s) {
    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(16, 18, 22));
    const int left = 58;
    const int right = width - 18;
    const int top = 42;
    const int bottom = height - 25;
    double maximum = 1.0;
    for (double value : event_history) maximum = std::max(maximum, value);
    for (double value : rgb_history) maximum = std::max(maximum, value);
    maximum *= 1.15;

    for (int grid = 0; grid <= 4; ++grid) {
        const int y = top + (bottom - top) * grid / 4;
        cv::line(plot, cv::Point(left, y), cv::Point(right, y), cv::Scalar(48, 52, 58), 1);
        const double value = maximum * (4 - grid) / 4.0;
        cv::putText(plot, cv::format("%.0f", value), cv::Point(7, y + 5), cv::FONT_HERSHEY_SIMPLEX,
                    0.38, cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
    }

    auto draw_series = [&](const std::deque<double> &history, cv::Scalar color) {
        if (history.size() < 2) return;
        for (std::size_t i = 1; i < history.size(); ++i) {
            const int x0 = left + static_cast<int>((i - 1) * (right - left) / 119.0);
            const int x1 = left + static_cast<int>(i * (right - left) / 119.0);
            const int y0 = bottom - static_cast<int>(std::min(maximum, history[i - 1]) / maximum * (bottom - top));
            const int y1 = bottom - static_cast<int>(std::min(maximum, history[i]) / maximum * (bottom - top));
            cv::line(plot, cv::Point(x0, y0), cv::Point(x1, y1), color, 2, cv::LINE_AA);
        }
    };
    draw_series(event_history, cv::Scalar(60, 220, 255));
    draw_series(rgb_history, cv::Scalar(80, 255, 100));

    std::ostringstream title;
    title << std::fixed << std::setprecision(1) << "HOST DATA RATE   Event selected ROI " << event_mb_s
          << " MB/s     RGB full BGR frame " << rgb_mb_s << " MB/s";
    put_label(plot, title.str(), cv::Point(14, 27), 0.58, cv::Scalar(240, 240, 240), 1);
    cv::line(plot, cv::Point(width - 330, 21), cv::Point(width - 295, 21), cv::Scalar(60, 220, 255), 3);
    cv::line(plot, cv::Point(width - 145, 21), cv::Point(width - 110, 21), cv::Scalar(80, 255, 100), 3);
    cv::putText(plot, "MB/s", cv::Point(7, top - 8), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
    return plot;
}

cv::Mat make_stimulus(const Options &options, const Candidate &candidate, double barcode_x,
                      int speed, bool automatic, int speed_index) {
    cv::Mat canvas(options.stimulus_height, options.stimulus_width, CV_8UC3, cv::Scalar(255, 255, 255));
    const int y = (options.stimulus_height - candidate.image.rows) / 2;
    const int x = static_cast<int>(std::lround(barcode_x));
    const cv::Rect target(x, y, candidate.image.cols, candidate.image.rows);
    const cv::Rect screen(0, 0, canvas.cols, canvas.rows);
    const cv::Rect clipped = target & screen;
    if (clipped.area() > 0) {
        const cv::Rect source(clipped.x - target.x, clipped.y - target.y, clipped.width, clipped.height);
        candidate.image(source).copyTo(canvas(clipped));
    }
    cv::rectangle(canvas, cv::Rect(0, 0, canvas.cols, 92), cv::Scalar(245, 245, 245), cv::FILLED);
    put_label(canvas, "EVENT CAMERA CODE 128  |  ID " + candidate.code.text,
              cv::Point(45, 58), 1.15, cv::Scalar(20, 20, 20), 2);
    const std::string mode = automatic ? "AUTO" : "MANUAL";
    put_label(canvas, mode + "  speed " + std::to_string(speed) + " px/s  [level " +
              std::to_string(speed_index + 1) + "]", cv::Point(canvas.cols - 690, 58), 0.82,
              cv::Scalar(50, 50, 50), 2);
    return canvas;
}

void save_diagnostic(const std::filesystem::path &directory, const cv::Mat &image, const std::string &name) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    cv::imwrite((directory / name).string(), image);
}

int run_demo(const Options &options) {
    // Equal-length payloads prevent a shorter template from matching a subset
    // of a longer barcode while providing varied Code 128 examples.
    const std::vector<std::string> payloads{
        "EVENT42", "SILKY01", "FAST128", "BOX2048",
        "TRACK99", "JET9000", "CODE777", "RAPID88", "SCAN555"
    };
    std::vector<Candidate> candidates;
    const int barcode_height = static_cast<int>(options.stimulus_height * 0.60);
    for (const auto &payload : payloads) {
        Candidate candidate;
        candidate.code = barcode::encode_code128_b(payload);
        candidate.image = barcode::render_code128(candidate.code, options.module_pixels, barcode_height);
        candidates.push_back(std::move(candidate));
    }
    // One warm-up speed followed by increasingly fast display motion.
    const std::vector<int> speeds{800, 1200, 1600, 2000, 2400};
    auto catalog_group = [&candidates](int index) {
        const int first = (index / 3) * 3;
        return std::vector<Candidate>(candidates.begin() + first, candidates.begin() + first + 3);
    };

    const std::filesystem::path output_dir(options.output_dir);
    std::error_code filesystem_error;
    std::filesystem::create_directories(output_dir, filesystem_error);
    const auto log_name = options.rgb_enabled && !options.stimulus_only ? "comparison_trials.csv" : "trials.csv";
    std::ofstream log(output_dir / log_name, std::ios::app);
    if (log.tellp() == 0) {
        log << "pass,payload,speed_px_s,event_decoded,event_success,event_score,event_latency_ms,"
               "rgb_decoded,rgb_success,rgb_latency_ms,rgb_frames,event_count,elapsed_s\n";
    }

    if (!options.headless) {
        cv::namedWindow("Barcode Stimulus", cv::WINDOW_NORMAL);
        cv::resizeWindow("Barcode Stimulus", options.stimulus_width, options.stimulus_height);
        // Do not request WINDOW_FULLSCREEN here. GNOME places fullscreen XWayland
        // windows back on the primary monitor regardless of moveWindow. A normal
        // window at the HDMI monitor's absolute desktop origin stays on HDMI.
        cv::moveWindow("Barcode Stimulus", options.stimulus_x, options.stimulus_y);
    }

    std::unique_ptr<Metavision::Camera> camera;
    std::unique_ptr<EventSurface> surface;
    std::unique_ptr<RgbCapture> rgb_camera;
    int camera_width = 640;
    int camera_height = 480;
    if (!options.stimulus_only) {
        if (options.input.empty()) camera = std::make_unique<Metavision::Camera>(Metavision::Camera::from_first_available());
        else camera = std::make_unique<Metavision::Camera>(Metavision::Camera::from_file(options.input));
        camera_width = camera->geometry().get_width();
        camera_height = camera->geometry().get_height();
        if (auto *biases = camera->get_device().get_facility<Metavision::I_LL_Biases>()) {
            const int old_on = biases->get("bias_diff_on");
            const int old_off = biases->get("bias_diff_off");
            const bool on_set = biases->set("bias_diff_on", 374);
            const bool off_set = biases->set("bias_diff_off", 234);
            std::cout << "Sensitivity biases: ON " << old_on << " -> " << biases->get("bias_diff_on")
                      << " (" << (on_set ? "ok" : "rejected") << "), OFF " << old_off << " -> "
                      << biases->get("bias_diff_off") << " (" << (off_set ? "ok" : "rejected") << ")\n";
        }
        surface = std::make_unique<EventSurface>(camera_width, camera_height);
        camera->cd().add_callback([&surface](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
            surface->ingest(begin, end);
        });
        if (!options.headless) {
            cv::namedWindow("Event vs RGB Barcode Reader", cv::WINDOW_NORMAL);
            cv::moveWindow("Event vs RGB Barcode Reader", 20, 40);
            cv::resizeWindow("Event vs RGB Barcode Reader", 1500, options.data_rate_plot ? 940 : 760);
        }
        camera->start();
        std::cout << "Camera running at " << camera_width << 'x' << camera_height << "\n";
        if (options.rgb_enabled) {
            rgb_camera = open_realsense_rgb(options.rgb_device);
            std::cout << "RealSense RGB auto-selected /dev/video" << rgb_camera->device_index()
                      << " at 640x480, " << rgb_camera->fps()
                      << " fps, exposure control " << rgb_camera->exposure() << "\n";
        }
    }

    bool automatic = true;
    bool running = true;
    int speed_index = 0;
    int payload_index = 0;
    std::vector<Candidate> active_catalog = catalog_group(payload_index);
    int pass = 1;
    int successes = 0;
    int rgb_successes = 0;
    std::string decoded_this_pass;
    double best_score_this_pass = 0.0;
    double best_raw_score_this_pass = 0.0;
    double best_raw_margin_this_pass = 0.0;
    std::string best_raw_text_this_pass;
    double barcode_x = -candidates[payload_index].image.cols;
    auto start_time = Clock::now();
    auto previous_time = start_time;
    auto pass_start = start_time;
    auto last_report = start_time;
    auto blank_until = start_time;
    std::uint64_t last_events = 0;
    std::uint64_t last_roi_events = 0;
    std::uint64_t last_rgb_sequence_rate = 0;
    std::uint64_t current_rgb_sequence = 0;
    double event_rate = 0.0;
    double event_roi_mb_s = 0.0;
    double rgb_full_mb_s = 0.0;
    std::deque<double> event_rate_history;
    std::deque<double> rgb_rate_history;
    std::string stable_text;
    int stable_count = 0;
    Match latest_match;
    cv::Mat latest_dashboard;
    std::size_t best_active_pixels = 0;
    double calibrated_scale = 0.0;
    std::deque<std::string> recent_matches;
    bool high_confidence_lock = false;
    std::vector<double> event_track_evidence(candidates.size(), 0.0);
    std::vector<int> event_track_observations(candidates.size(), 0);
    std::string rgb_decoded_this_pass;
    std::string rgb_last_read;
    double event_latency_ms = -1.0;
    double rgb_latency_ms = -1.0;
    std::uint64_t last_rgb_sequence = 0;
    std::uint64_t rgb_frames_this_pass = 0;
    cv::Mat latest_rgb_frame;

    while (running) {
        const auto now = Clock::now();
        const double dt = std::min(0.1, std::chrono::duration<double>(now - previous_time).count());
        previous_time = now;
        const bool blank = now < blank_until;
        const int speed = speeds[speed_index];
        if (!blank) barcode_x += speed * dt;

        auto stimulus = make_stimulus(options, candidates[payload_index], blank ? -10000.0 : barcode_x,
                                      speed, automatic, speed_index);
        if (!options.headless) cv::imshow("Barcode Stimulus", stimulus);

        if (rgb_camera) {
            auto rgb_snapshot = rgb_camera->snapshot();
            if (rgb_snapshot.sequence != 0 && rgb_snapshot.sequence != last_rgb_sequence) {
                last_rgb_sequence = rgb_snapshot.sequence;
                current_rgb_sequence = rgb_snapshot.sequence;
                ++rgb_frames_this_pass;
                latest_rgb_frame = std::move(rgb_snapshot.frame);
                if (!blank && !rgb_snapshot.decoded.empty()) {
                        rgb_last_read = rgb_snapshot.decoded;
                        if (rgb_snapshot.decoded == candidates[payload_index].code.text && rgb_decoded_this_pass.empty()) {
                            rgb_decoded_this_pass = rgb_snapshot.decoded;
                            rgb_latency_ms = std::chrono::duration<double, std::milli>(now - pass_start).count();
                        }
                }
            }
        }

        EventSurface::Snapshot snapshot;
        if (surface) {
            snapshot = surface->snapshot();
            auto processed = process_snapshot(snapshot, camera_width, camera_height,
                                              options.persistence_us, options.decode_window_us);
            latest_match = match_known_codes(processed.profile, processed.on_profile, processed.off_profile,
                                             active_catalog, calibrated_scale);
            if (latest_match.valid && !blank) {
                surface->set_roi(latest_match.left, processed.roi_top, latest_match.right,
                                 processed.roi_bottom + 1);
            } else {
                surface->clear_roi();
            }
            if (latest_match.score > best_raw_score_this_pass) {
                best_raw_score_this_pass = latest_match.score;
                best_raw_margin_this_pass = latest_match.margin;
                best_raw_text_this_pass = latest_match.text;
            }

            // Temporal barcode tracker. Each event slice contributes weighted
            // evidence instead of having to pass a hard threshold by itself.
            // This keeps tracking through incomplete LCD refreshes and fast
            // motion while still requiring agreement over multiple slices.
            for (double &evidence : event_track_evidence) evidence *= 0.992;
            if (!latest_match.text.empty() && latest_match.score > 0.18) {
                for (std::size_t i = 0; i < candidates.size(); ++i) {
                    if (candidates[i].code.text != latest_match.text) continue;
                    const double weight = std::max(0.0, latest_match.score - 0.16) *
                                          (1.0 + 5.0 * std::max(0.0, latest_match.margin));
                    event_track_evidence[i] += weight;
                    ++event_track_observations[i];
                    break;
                }
                const auto tracked = static_cast<std::size_t>(std::distance(event_track_evidence.begin(),
                    std::max_element(event_track_evidence.begin(), event_track_evidence.end())));
                double runner_up_evidence = 0.0;
                for (std::size_t i = 0; i < event_track_evidence.size(); ++i) {
                    if (i != tracked) runner_up_evidence = std::max(runner_up_evidence, event_track_evidence[i]);
                }
                if (!high_confidence_lock && event_track_observations[tracked] >= 3 &&
                    event_track_evidence[tracked] > 0.32 &&
                    event_track_evidence[tracked] > runner_up_evidence + 0.045) {
                    decoded_this_pass = candidates[tracked].code.text;
                    best_score_this_pass = std::max(best_score_this_pass, latest_match.score);
                }
            }
            if (latest_match.valid) {
                const auto measured_edges = extract_edges(processed.profile, latest_match.left, latest_match.right);
                const auto generic_decode = barcode::decode_edge_positions(measured_edges);
                if (generic_decode.valid && generic_decode.text == latest_match.text) {
                    latest_match.score = std::min(1.0, latest_match.score + 0.12);
                }
            }
            if (latest_match.valid) {
                if (latest_match.margin > 0.04 && latest_match.score > 0.30) {
                    calibrated_scale = calibrated_scale == 0.0 ? latest_match.module_scale :
                        0.90 * calibrated_scale + 0.10 * latest_match.module_scale;
                }
                if (latest_match.text == stable_text) ++stable_count;
                else { stable_text = latest_match.text; stable_count = 1; }
                if (latest_match.margin > 0.08 && latest_match.score > 0.40) {
                    decoded_this_pass = latest_match.text;
                    best_score_this_pass = latest_match.score;
                    high_confidence_lock = true;
                }
                recent_matches.push_back(latest_match.text);
                if (recent_matches.size() > 15) recent_matches.pop_front();
                std::vector<int> votes(candidates.size(), 0);
                for (const auto &text : recent_matches) {
                    for (std::size_t i = 0; i < candidates.size(); ++i) {
                        if (text == candidates[i].code.text) ++votes[i];
                    }
                }
                const auto winner = static_cast<std::size_t>(std::distance(votes.begin(),
                    std::max_element(votes.begin(), votes.end())));
                int runner_up = 0;
                for (std::size_t i = 0; i < votes.size(); ++i) if (i != winner) runner_up = std::max(runner_up, votes[i]);
                if (!high_confidence_lock && votes[winner] >= 5 && votes[winner] >= runner_up + 2) {
                    decoded_this_pass = candidates[winner].code.text;
                    best_score_this_pass = std::max(best_score_this_pass, latest_match.score);
                }
            } else {
                stable_count = std::max(0, stable_count - 1);
            }
            if (decoded_this_pass == candidates[payload_index].code.text && event_latency_ms < 0.0) {
                event_latency_ms = std::chrono::duration<double, std::milli>(now - pass_start).count();
            }

            const auto report_elapsed = std::chrono::duration<double>(now - last_report).count();
            if (report_elapsed >= 1.0) {
                event_rate = (snapshot.total_events - last_events) / report_elapsed;
                const double roi_events_per_second = (snapshot.roi_events - last_roi_events) / report_elapsed;
                event_roi_mb_s = roi_events_per_second * sizeof(Metavision::EventCD) / 1e6;
                const std::uint64_t rgb_frames = current_rgb_sequence - last_rgb_sequence_rate;
                const double rgb_frame_bytes = latest_rgb_frame.empty() ? 640.0 * 480.0 * 3.0 :
                    static_cast<double>(latest_rgb_frame.total() * latest_rgb_frame.elemSize());
                rgb_full_mb_s = rgb_frames * rgb_frame_bytes / report_elapsed / 1e6;
                last_events = snapshot.total_events;
                last_roi_events = snapshot.roi_events;
                last_rgb_sequence_rate = current_rgb_sequence;
                last_report = now;
                event_rate_history.push_back(event_roi_mb_s);
                rgb_rate_history.push_back(rgb_full_mb_s);
                if (event_rate_history.size() > 120) event_rate_history.pop_front();
                if (rgb_rate_history.size() > 120) rgb_rate_history.pop_front();
            }

            const int graph_height = 90;
            cv::Mat graph(graph_height, camera_width, CV_8UC3, cv::Scalar(18, 18, 18));
            const float profile_peak = processed.profile.empty() ? 1.0f :
                std::max(1.0f, *std::max_element(processed.profile.begin(), processed.profile.end()));
            for (int x = 1; x < camera_width; ++x) {
                const int y0 = graph_height - 1 - static_cast<int>((processed.profile[x - 1] / profile_peak) * (graph_height - 8));
                const int y1 = graph_height - 1 - static_cast<int>((processed.profile[x] / profile_peak) * (graph_height - 8));
                cv::line(graph, cv::Point(x - 1, y0), cv::Point(x, y1), cv::Scalar(80, 230, 255), 1);
            }
            cv::line(processed.image, cv::Point(0, processed.roi_top), cv::Point(camera_width - 1, processed.roi_top),
                     cv::Scalar(120, 120, 120), 1);
            cv::line(processed.image, cv::Point(0, processed.roi_bottom), cv::Point(camera_width - 1, processed.roi_bottom),
                     cv::Scalar(120, 120, 120), 1);
            if (latest_match.valid) {
                cv::rectangle(processed.image,
                              cv::Rect(latest_match.left, processed.roi_top,
                                       std::max(1, latest_match.right - latest_match.left),
                                       processed.roi_bottom - processed.roi_top + 1),
                              cv::Scalar(60, 220, 60), 2);
            }

            cv::Mat event_dashboard;
            cv::vconcat(processed.image, graph, event_dashboard);
            const bool currently_correct = decoded_this_pass == candidates[payload_index].code.text;
            const cv::Scalar status_color = currently_correct ? cv::Scalar(60, 255, 60) : cv::Scalar(80, 210, 255);
            put_label(event_dashboard, "SILKYCAM EVENT CAMERA", cv::Point(14, 30), 0.72,
                      cv::Scalar(255, 255, 255), 2);
            const std::string decode_label = decoded_this_pass.empty() ? "SCANNING" : "DECODED " + decoded_this_pass;
            put_label(event_dashboard, decode_label, cv::Point(14, 64), 0.82, status_color, 2);
            std::ostringstream metrics;
            metrics << std::fixed << std::setprecision(1) << "events " << event_rate / 1e6 << " Mev/s"
                    << "  speed " << speed << " px/s  confidence " << latest_match.score
                    << "  pass " << pass << "  reads " << successes;
            put_label(event_dashboard, metrics.str(), cv::Point(14, camera_height - 16), 0.48,
                      cv::Scalar(235, 235, 235), 1);

            cv::Mat rgb_top(camera_height, camera_width, CV_8UC3, cv::Scalar(20, 20, 20));
            if (!latest_rgb_frame.empty()) cv::resize(latest_rgb_frame, rgb_top, rgb_top.size());
            cv::Mat rgb_footer(graph_height, camera_width, CV_8UC3, cv::Scalar(18, 18, 18));
            cv::Mat rgb_dashboard;
            cv::vconcat(rgb_top, rgb_footer, rgb_dashboard);
            const bool rgb_correct = rgb_decoded_this_pass == candidates[payload_index].code.text;
            const std::string rgb_label = rgb_correct ? "DECODED " + rgb_decoded_this_pass :
                (rgb_last_read.empty() ? "SCANNING" : "LAST " + rgb_last_read);
            put_label(rgb_dashboard, "REALSENSE RGB  640x480 @ 60 FPS", cv::Point(14, 30), 0.65,
                      cv::Scalar(255, 255, 255), 2);
            put_label(rgb_dashboard, rgb_label, cv::Point(14, 64), 0.82,
                      rgb_correct ? cv::Scalar(60, 255, 60) : cv::Scalar(80, 210, 255), 2);
            std::ostringstream rgb_metrics;
            rgb_metrics << "speed " << speed << " px/s  frames " << rgb_frames_this_pass
                        << "  pass " << pass << "  reads " << rgb_successes;
            put_label(rgb_dashboard, rgb_metrics.str(), cv::Point(14, camera_height + 55), 0.48,
                      cv::Scalar(235, 235, 235), 1);
            cv::hconcat(event_dashboard, rgb_dashboard, latest_dashboard);
            if (options.data_rate_plot) {
                const auto data_rate_plot = render_data_rate_plot(latest_dashboard.cols, 180,
                                                                  event_rate_history, rgb_rate_history,
                                                                  event_roi_mb_s, rgb_full_mb_s);
                cv::Mat dashboard_with_plot;
                cv::vconcat(latest_dashboard, data_rate_plot, dashboard_with_plot);
                latest_dashboard = std::move(dashboard_with_plot);
            }
            if (processed.active_pixels > best_active_pixels) {
                best_active_pixels = processed.active_pixels;
                save_diagnostic(output_dir, latest_dashboard, "peak_activity.png");
            }
            if (!options.headless) cv::imshow("Event vs RGB Barcode Reader", latest_dashboard);
        }

        if (barcode_x > options.stimulus_width) {
            const bool success_this_pass = decoded_this_pass == candidates[payload_index].code.text;
            const bool rgb_success_this_pass = rgb_decoded_this_pass == candidates[payload_index].code.text;
            if (success_this_pass) ++successes;
            if (rgb_success_this_pass) ++rgb_successes;
            const double elapsed = std::chrono::duration<double>(now - start_time).count();
            log << pass << ',' << candidates[payload_index].code.text << ',' << speed << ','
                << decoded_this_pass << ',' << (success_this_pass ? 1 : 0) << ','
                << best_score_this_pass << ',' << event_latency_ms << ','
                << (rgb_decoded_this_pass.empty() ? rgb_last_read : rgb_decoded_this_pass) << ','
                << (rgb_success_this_pass ? 1 : 0) << ',' << rgb_latency_ms << ','
                << rgb_frames_this_pass << ',' << snapshot.total_events << ',' << elapsed << '\n';
            log.flush();
            std::cout << "pass=" << pass << " expected=" << candidates[payload_index].code.text
                      << " speed=" << speed << " event=" << (decoded_this_pass.empty() ? "-" : decoded_this_pass)
                      << '(' << (success_this_pass ? "PASS" : "MISS") << ") rgb="
                      << (rgb_decoded_this_pass.empty() ? (rgb_last_read.empty() ? "-" : rgb_last_read) : rgb_decoded_this_pass)
                      << '(' << (rgb_success_this_pass ? "PASS" : "MISS") << ")\n";
            std::cout << "  raw_best=" << best_raw_text_this_pass << " raw_score=" << best_raw_score_this_pass
                      << " margin=" << best_raw_margin_this_pass << '\n';
            if (!latest_dashboard.empty()) save_diagnostic(output_dir, latest_dashboard, "latest_comparison.png");
            ++pass;
            payload_index = (payload_index + 1) % static_cast<int>(candidates.size());
            active_catalog = catalog_group(payload_index);
            if (automatic) speed_index = (speed_index + 1) % static_cast<int>(speeds.size());
            barcode_x = -candidates[payload_index].image.cols;
            decoded_this_pass.clear();
            best_score_this_pass = 0.0;
            best_raw_score_this_pass = 0.0;
            best_raw_margin_this_pass = 0.0;
            best_raw_text_this_pass.clear();
            stable_text.clear();
            stable_count = 0;
            recent_matches.clear();
            high_confidence_lock = false;
            std::fill(event_track_evidence.begin(), event_track_evidence.end(), 0.0);
            std::fill(event_track_observations.begin(), event_track_observations.end(), 0);
            rgb_decoded_this_pass.clear();
            rgb_last_read.clear();
            event_latency_ms = -1.0;
            rgb_latency_ms = -1.0;
            rgb_frames_this_pass = 0;
            pass_start = now;
            blank_until = now + std::chrono::milliseconds(450);
        }

        int key = -1;
        if (!options.headless) key = cv::waitKey(1) & 0xff;
        else std::this_thread::sleep_for(std::chrono::milliseconds(4));
        if (key == 27 || key == 'q' || key == 'Q') running = false;
        else if (key == ' ') automatic = !automatic;
        else if (key >= '1' && key <= '5') { speed_index = key - '1'; automatic = false; }
        else if ((key == 's' || key == 'S') && !latest_dashboard.empty()) {
            save_diagnostic(output_dir, latest_dashboard, "manual_snapshot.png");
        }

        if (camera && !camera->is_running()) running = false;
        if (options.duration_seconds > 0 &&
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() >= options.duration_seconds) {
            running = false;
        }
        (void)pass_start;
    }

    if (rgb_camera) rgb_camera->stop();
    if (camera) camera->stop();
    if (!latest_dashboard.empty()) save_diagnostic(output_dir, latest_dashboard, "final_comparison.png");
    if (!options.headless) cv::destroyAllWindows();
    std::cout << "Demo stopped after " << pass - 1 << " completed passes; event=" << successes
              << " reads, RGB=" << rgb_successes << " reads.\n";
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    try {
        const Options options = parse_options(argc, argv);
        if (options.self_test) {
            std::string message;
            const bool passed = barcode::self_test(message);
            std::cout << (passed ? "PASS: " : "FAIL: ") << message << '\n';
            return passed ? 0 : 1;
        }
        return run_demo(options);
    } catch (const std::exception &error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}

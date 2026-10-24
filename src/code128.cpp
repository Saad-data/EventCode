#include "code128.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace barcode {
namespace {

// ISO/IEC 15417 Code 128 element widths. Values 103-105 are starts and 106 is stop.
constexpr std::array<const char *, 107> kPatterns = {{
    "212222", "222122", "222221", "121223", "121322", "131222", "122213", "122312", "132212",
    "221213", "221312", "231212", "112232", "122132", "122231", "113222", "123122", "123221",
    "223211", "221132", "221231", "213212", "223112", "312131", "311222", "321122", "321221",
    "312212", "322112", "322211", "212123", "212321", "232121", "111323", "131123", "131321",
    "112313", "132113", "132311", "211313", "231113", "231311", "112133", "112331", "132131",
    "113123", "113321", "133121", "313121", "211331", "231131", "213113", "213311", "213131",
    "311123", "311321", "331121", "312113", "312311", "332111", "314111", "221411", "431111",
    "111224", "111422", "121124", "121421", "141122", "141221", "112214", "112412", "122114",
    "122411", "142112", "142211", "241211", "221114", "413111", "241112", "134111", "111242",
    "121142", "121241", "114212", "124112", "124211", "411212", "421112", "421211", "212141",
    "214121", "412121", "111143", "111341", "131141", "114113", "114311", "411113", "411311",
    "113141", "114131", "311141", "411131", "211412", "211214", "211232", "2331112"
}};

std::vector<int> pattern_runs(int symbol) {
    std::vector<int> out;
    for (const char *p = kPatterns.at(static_cast<std::size_t>(symbol)); *p; ++p) out.push_back(*p - '0');
    return out;
}

double pattern_error(const std::vector<double> &widths, std::size_t pos, int symbol) {
    const auto expected = pattern_runs(symbol);
    if (pos + expected.size() > widths.size()) return 1e9;
    const double measured_sum = std::accumulate(widths.begin() + static_cast<long>(pos),
                                                widths.begin() + static_cast<long>(pos + expected.size()), 0.0);
    const double expected_sum = std::accumulate(expected.begin(), expected.end(), 0.0);
    if (measured_sum <= 0.0) return 1e9;
    const double scale = measured_sum / expected_sum;
    double err = 0.0;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const double d = widths[pos + i] / scale - expected[i];
        err += d * d;
    }
    return std::sqrt(err / expected.size());
}

int best_symbol(const std::vector<double> &widths, std::size_t pos, int first, int last, double &error) {
    int best = -1;
    error = 1e9;
    for (int symbol = first; symbol <= last; ++symbol) {
        const double e = pattern_error(widths, pos, symbol);
        if (e < error) { error = e; best = symbol; }
    }
    return best;
}

DecodeResult decode_forward(const std::vector<double> &edges) {
    DecodeResult result;
    if (edges.size() < 21) return result;

    std::vector<double> widths;
    widths.reserve(edges.size() - 1);
    for (std::size_t i = 1; i < edges.size(); ++i) {
        if (edges[i] <= edges[i - 1]) return result;
        widths.push_back(edges[i] - edges[i - 1]);
    }

    double start_error = 0.0;
    const int start = best_symbol(widths, 0, 103, 105, start_error);
    if (start_error > 0.72) return result;

    std::vector<int> symbols{start};
    double total_error = start_error;
    std::size_t pos = 6;
    while (pos + 7 <= widths.size() && symbols.size() < 40) {
        const double stop_error = pattern_error(widths, pos, 106);
        if (pos + 7 == widths.size() && stop_error < 0.72) {
            symbols.push_back(106);
            total_error += stop_error;
            pos += 7;
            break;
        }
        double symbol_error = 0.0;
        const int symbol = best_symbol(widths, pos, 0, 102, symbol_error);
        if (symbol_error > 0.78) return result;
        symbols.push_back(symbol);
        total_error += symbol_error;
        pos += 6;
    }

    if (symbols.size() < 4 || symbols.back() != 106 || pos != widths.size()) return result;
    const int check = symbols[symbols.size() - 2];
    int sum = symbols.front();
    for (std::size_t i = 1; i + 2 < symbols.size(); ++i) sum += symbols[i] * static_cast<int>(i);
    if (sum % 103 != check) return result;

    std::ostringstream decoded;
    int active_set = start;
    for (std::size_t i = 1; i + 2 < symbols.size(); ++i) {
        const int value = symbols[i];
        if (value == 100) { active_set = 104; continue; }
        if (value == 101) { active_set = 103; continue; }
        if (active_set != 104 || value < 0 || value > 94) return result;
        decoded << static_cast<char>(value + 32);
    }
    result.valid = true;
    result.text = decoded.str();
    result.error = total_error / static_cast<double>(symbols.size() - 1);
    return result;
}

} // namespace

EncodedCode128 encode_code128_b(const std::string &text) {
    if (text.empty()) throw std::invalid_argument("Code 128 payload must not be empty");
    EncodedCode128 code;
    code.text = text;
    code.symbols.push_back(104);
    int checksum = 104;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        if (c < 32 || c > 126) throw std::invalid_argument("Code 128-B accepts printable ASCII only");
        const int value = static_cast<int>(c) - 32;
        code.symbols.push_back(value);
        checksum += value * static_cast<int>(i + 1);
    }
    code.symbols.push_back(checksum % 103);
    code.symbols.push_back(106);

    int module = 0;
    code.edge_modules.push_back(0);
    for (const int symbol : code.symbols) {
        for (const int width : pattern_runs(symbol)) {
            code.runs.push_back(width);
            module += width;
            code.edge_modules.push_back(module);
        }
    }
    code.modules = module;
    return code;
}

cv::Mat render_code128(const EncodedCode128 &code, int module_px, int height_px, int quiet_modules) {
    const int width = (code.modules + 2 * quiet_modules) * module_px;
    cv::Mat image(height_px, width, CV_8UC3, cv::Scalar(255, 255, 255));
    int x = quiet_modules * module_px;
    bool black = true;
    for (const int run : code.runs) {
        const int pixels = run * module_px;
        if (black) cv::rectangle(image, cv::Rect(x, 0, pixels, height_px), cv::Scalar(0, 0, 0), cv::FILLED);
        x += pixels;
        black = !black;
    }
    return image;
}

DecodeResult decode_edge_positions(const std::vector<double> &edges) {
    auto forward = decode_forward(edges);
    std::vector<double> reversed;
    reversed.reserve(edges.size());
    if (!edges.empty()) {
        const double right = edges.back();
        for (auto it = edges.rbegin(); it != edges.rend(); ++it) reversed.push_back(right - *it);
    }
    auto backward = decode_forward(reversed);
    backward.reversed = backward.valid;
    if (!forward.valid) return backward;
    if (!backward.valid || forward.error <= backward.error) return forward;
    return backward;
}

DecodeResult decode_grayscale_frame(const cv::Mat &frame, int expected_payload_length) {
    DecodeResult best;
    if (frame.empty()) return best;
    cv::Mat gray;
    if (frame.channels() == 1) gray = frame;
    else cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Decode several independent horizontal scan lines. This is a conventional
    // frame-camera Code 128 decoder: it measures black/white run widths and
    // accepts only payloads with a valid Code 128 checksum.
    for (int row_index = 0; row_index < 11; ++row_index) {
        const int y = static_cast<int>(gray.rows * (0.28 + row_index * 0.04));
        if (y < 0 || y >= gray.rows) continue;
        cv::Mat smooth;
        cv::GaussianBlur(gray.row(y), smooth, cv::Size(5, 1), 0.0);
        cv::Mat binary;
        cv::threshold(smooth, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        struct Run { int begin; int end; bool black; };
        std::vector<Run> runs;
        bool black = binary.at<unsigned char>(0, 0) == 0;
        int begin = 0;
        for (int x = 1; x < binary.cols; ++x) {
            const bool current_black = binary.at<unsigned char>(0, x) == 0;
            if (current_black != black) {
                runs.push_back({begin, x, black});
                begin = x;
                black = current_black;
            }
        }
        runs.push_back({begin, binary.cols, black});

        for (std::size_t start = 0; start < runs.size(); ++start) {
            if (!runs[start].black) continue;
            const int first_length = expected_payload_length > 0 ? expected_payload_length : 1;
            const int last_length = expected_payload_length > 0 ? expected_payload_length : 20;
            for (int payload_length = first_length; payload_length <= last_length; ++payload_length) {
                const std::size_t run_count = static_cast<std::size_t>(6 * payload_length + 19);
                if (start + run_count > runs.size()) break;
                if (!runs[start + run_count - 1].black) continue;
                std::vector<double> edges;
                edges.reserve(run_count + 1);
                edges.push_back(runs[start].begin);
                for (std::size_t r = 0; r < run_count; ++r) edges.push_back(runs[start + r].end);
                auto decoded = decode_edge_positions(edges);
                if (decoded.valid && decoded.error < best.error) best = std::move(decoded);
            }
        }
    }
    return best;
}

bool self_test(std::string &message) {
    try {
        for (const std::string text : {"EVENT42", "SILKY01", "FAST128", "HELLO-123"}) {
            const auto code = encode_code128_b(text);
            std::vector<double> edges(code.edge_modules.begin(), code.edge_modules.end());
            auto decoded = decode_edge_positions(edges);
            if (!decoded.valid || decoded.text != text) {
                message = "round-trip failed for " + text;
                return false;
            }
            const auto image_decoded = decode_grayscale_frame(render_code128(code, 5, 220));
            if (!image_decoded.valid || image_decoded.text != text) {
                message = "image decode failed for " + text;
                return false;
            }
        }
        message = "Code 128 generator, checksum and edge decoder passed";
        return true;
    } catch (const std::exception &e) {
        message = e.what();
        return false;
    }
}

} // namespace barcode

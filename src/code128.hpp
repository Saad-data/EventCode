#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace barcode {

struct EncodedCode128 {
    std::string text;
    std::vector<int> symbols;
    std::vector<int> runs;
    std::vector<int> edge_modules;
    int modules = 0;
};

struct DecodeResult {
    bool valid = false;
    std::string text;
    double error = 1e9;
    bool reversed = false;
};

EncodedCode128 encode_code128_b(const std::string &text);
cv::Mat render_code128(const EncodedCode128 &code, int module_px, int height_px, int quiet_modules = 12);
DecodeResult decode_edge_positions(const std::vector<double> &edges);
DecodeResult decode_grayscale_frame(const cv::Mat &frame, int expected_payload_length = -1);
bool self_test(std::string &message);

} // namespace barcode

#include <libobsensor/ObSensor.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace {

struct Args {
    fs::path output_dir = "output";
    int      width      = 1280;
    int      height     = 720;
    int      fps        = 30;
    int      warmup     = 20;
    double   timeout_s  = 20.0;
};

void printUsage(const char *program) {
    std::cout << "Usage: " << program
              << " [--output DIR] [--width N] [--height N] [--fps N]"
              << " [--warmup N] [--timeout SEC]\n";
}

Args parseArgs(int argc, char **argv) {
    Args args;
    for(int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if(key == "--help" || key == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if(i + 1 >= argc) {
            throw std::invalid_argument("Missing value for " + key);
        }
        const std::string value = argv[++i];
        if(key == "--output") {
            args.output_dir = value;
        }
        else if(key == "--width") {
            args.width = std::stoi(value);
        }
        else if(key == "--height") {
            args.height = std::stoi(value);
        }
        else if(key == "--fps") {
            args.fps = std::stoi(value);
        }
        else if(key == "--warmup") {
            args.warmup = std::stoi(value);
        }
        else if(key == "--timeout") {
            args.timeout_s = std::stod(value);
        }
        else {
            throw std::invalid_argument("Unknown argument: " + key);
        }
    }
    if(args.width <= 0 || args.height <= 0 || args.fps <= 0 || args.warmup < 0 || args.timeout_s <= 0.0) {
        throw std::invalid_argument(
            "width, height, fps and timeout must be positive; warmup must be non-negative"
        );
    }
    return args;
}

std::string jsonEscape(const std::string &value) {
    std::string escaped;
    escaped.reserve(value.size());
    for(const char ch: value) {
        switch(ch) {
        case '\\': escaped += "\\\\"; break;
        case '"': escaped += "\\\""; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default: escaped += ch; break;
        }
    }
    return escaped;
}

const char *distortionModelName(OBCameraDistortionModel model) {
    switch(model) {
    case OB_DISTORTION_NONE: return "none";
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY: return "modified_brown_conrady";
    case OB_DISTORTION_INVERSE_BROWN_CONRADY: return "inverse_brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY: return "brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY_K6: return "brown_conrady_k6";
    case OB_DISTORTION_KANNALA_BRANDT4: return "kannala_brandt4";
    default: return "unknown";
    }
}

void writeMetadata(const fs::path                  &path,
                   const std::shared_ptr<ob::DeviceInfo> &device_info,
                   const OBCameraIntrinsic         &intrinsic,
                   const OBCameraDistortion        &distortion,
                   const std::shared_ptr<ob::ColorFrame> &color,
                   const std::shared_ptr<ob::DepthFrame> &depth) {
    std::ofstream out(path);
    if(!out) {
        throw std::runtime_error("Cannot write metadata: " + path.string());
    }
    out << std::fixed << std::setprecision(10);
    out << "{\n";
    out << "  \"schema_version\": 1,\n";
    out << "  \"device\": {\n";
    out << "    \"name\": \"" << jsonEscape(device_info->getName()) << "\",\n";
    out << "    \"serial_number\": \"" << jsonEscape(device_info->getSerialNumber()) << "\",\n";
    out << "    \"firmware_version\": \"" << jsonEscape(device_info->getFirmwareVersion()) << "\",\n";
    out << "    \"connection_type\": \"" << jsonEscape(device_info->getConnectionType()) << "\",\n";
    out << "    \"vid\": " << device_info->getVid() << ",\n";
    out << "    \"pid\": " << device_info->getPid() << "\n";
    out << "  },\n";
    out << "  \"files\": {\"color\": \"color.png\", \"depth\": \"depth.png\"},\n";
    out << "  \"alignment\": \"depth_to_color_software\",\n";
    out << "  \"camera_coordinate_convention\": \"color optical frame: +x right, +y down, +z forward\",\n";
    out << "  \"color\": {\n";
    out << "    \"width\": " << color->width() << ", \"height\": " << color->height() << ",\n";
    out << "    \"timestamp_ms\": " << color->timeStamp() << ",\n";
    out << "    \"intrinsics\": {\"fx\": " << intrinsic.fx << ", \"fy\": " << intrinsic.fy
        << ", \"cx\": " << intrinsic.cx << ", \"cy\": " << intrinsic.cy
        << ", \"width\": " << intrinsic.width << ", \"height\": " << intrinsic.height << "},\n";
    out << "    \"distortion\": {\n";
    out << "      \"model\": \"" << distortionModelName(distortion.model) << "\", \"model_id\": "
        << static_cast<int>(distortion.model) << ",\n";
    out << "      \"k1\": " << distortion.k1 << ", \"k2\": " << distortion.k2
        << ", \"k3\": " << distortion.k3 << ", \"k4\": " << distortion.k4
        << ", \"k5\": " << distortion.k5 << ", \"k6\": " << distortion.k6
        << ", \"p1\": " << distortion.p1 << ", \"p2\": " << distortion.p2 << "\n";
    out << "    }\n";
    out << "  },\n";
    out << "  \"depth\": {\n";
    out << "    \"width\": " << depth->width() << ", \"height\": " << depth->height() << ",\n";
    out << "    \"timestamp_ms\": " << depth->timeStamp() << ",\n";
    out << "    \"storage\": \"uint16_png\",\n";
    out << "    \"value_scale_mm_per_unit\": " << depth->getValueScale() << ",\n";
    out << "    \"scale_m_per_unit\": " << depth->getValueScale() / 1000.0f << "\n";
    out << "  }\n";
    out << "}\n";
}

}  // namespace

int main(int argc, char **argv) try {
    const Args args = parseArgs(argc, argv);
    fs::create_directories(args.output_dir);
    ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");
    ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_ERROR);

    auto pipeline = std::make_shared<ob::Pipeline>();
    auto config   = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, args.width, args.height, args.fps, OB_FORMAT_RGB);
    config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, args.fps, OB_FORMAT_ANY);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    pipeline->enableFrameSync();
    pipeline->start(config);

    auto align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    std::shared_ptr<ob::FrameSet> aligned;
    int matched_frames = 0;
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(args.timeout_s);
    while(matched_frames <= args.warmup && std::chrono::steady_clock::now() < deadline) {
        auto frameset = pipeline->waitForFrameset(1500);
        if(!frameset) {
            continue;
        }
        auto aligned_frame = align->process(frameset);
        if(!aligned_frame) {
            continue;
        }
        aligned = aligned_frame->as<ob::FrameSet>();
        ++matched_frames;
    }

    if(!aligned || matched_frames <= args.warmup) {
        throw std::runtime_error(
            "Timed out waiting for aligned Orbbec RGB-D frames"
        );
    }
    auto color_frame = aligned->getFrame(OB_FRAME_COLOR);
    auto depth_frame = aligned->getFrame(OB_FRAME_DEPTH);
    if(!color_frame || !depth_frame) {
        throw std::runtime_error("Aligned frameset does not contain both color and depth frames");
    }
    auto color = color_frame->as<ob::ColorFrame>();
    auto depth = depth_frame->as<ob::DepthFrame>();
    if(color->width() != depth->width() || color->height() != depth->height()) {
        throw std::runtime_error("Aligned depth dimensions do not match color dimensions");
    }

    auto color_profile = color->getStreamProfile()->as<ob::VideoStreamProfile>();
    const auto intrinsic  = color_profile->getIntrinsic();
    const auto distortion = color_profile->getDistortion();

    const cv::Mat rgb(color->height(), color->width(), CV_8UC3, color->data());
    cv::Mat       bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    const cv::Mat depth_u16(depth->height(), depth->width(), CV_16UC1, depth->data());

    const fs::path color_path = args.output_dir / "color.png";
    const fs::path depth_path = args.output_dir / "depth.png";
    if(!cv::imwrite(color_path.string(), bgr)) {
        throw std::runtime_error("Failed to save " + color_path.string());
    }
    if(!cv::imwrite(depth_path.string(), depth_u16, { cv::IMWRITE_PNG_COMPRESSION, 0 })) {
        throw std::runtime_error("Failed to save " + depth_path.string());
    }

    const auto device_info = pipeline->getDevice()->getDeviceInfo();
    writeMetadata(args.output_dir / "camera.json", device_info, intrinsic, distortion, color, depth);
    pipeline->stop();

    std::cout << "Captured " << device_info->getName() << " (SN " << device_info->getSerialNumber() << ")\n";
    std::cout << "Color/depth: " << color->width() << "x" << color->height() << "\n";
    std::cout << "Depth scale: " << depth->getValueScale() << " mm/unit\n";
    std::cout << "Output: " << fs::absolute(args.output_dir) << "\n";
    return 0;
}
catch(const ob::Error &error) {
    std::cerr << "OrbbecSDK error in " << error.getFunction() << ": " << error.what() << "\n";
    return 2;
}
catch(const std::exception &error) {
    std::cerr << "Error: " << error.what() << "\n";
    return 1;
}

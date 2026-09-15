#pragma once

#include "rm_image_compression/codec.hpp"

#include <string>

namespace rm_image_compression::detail
{

std::string ffmpeg_error_to_string(int errnum);
std::string resolve_preset(std::string requested);
std::string build_x264_params();

}  // namespace rm_image_compression::detail

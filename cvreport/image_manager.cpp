#include "image_manager.hpp"

fs::path ImageManager::root = "images";

std::tuple<fs::path, fs::path> ImageManager::GetStereoVideoPath(std::string basename)
{
	auto left_path = root / (basename + "_left.mp4");
	auto right_path = root / (basename + "_right.mp4");
	return std::make_tuple(left_path, right_path);
}

#ifndef CVREPORT_IMAGE_MANAGER_H_
#define CVREPORT_IMAGE_MANAGER_H_

#include <filesystem>
#include <string>
#include <tuple>

namespace fs = std::filesystem;

class ImageManager
{
public:
	static std::tuple<fs::path, fs::path> GetStereoVideoPath(std::string basename);
private:
	static fs::path root;
};

#endif  // CVREPORT_IMAGEMANAGER_H_

#ifndef UFOEXPLORER_ROBOT_MODEL_H
#define UFOEXPLORER_ROBOT_MODEL_H

namespace ufoexplorer
{
struct RobotModel {
	std::string frame_id;
	double radius;
	double height;
};
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_ROBOT_MODEL_H
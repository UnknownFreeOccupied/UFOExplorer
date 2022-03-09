#ifndef UFOEXPLORER_SENSOR_MODEL_H
#define UFOEXPLORER_SENSOR_MODEL_H

namespace ufoexplorer
{
struct SensorModel {
	double horizontal_fov;
	double vertical_fov;
	double range_min;
	double range_max;
};
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_SENSOR_MODEL_H
#ifndef UFOEXPLORER_TIMING_H
#define UFOEXPLORER_TIMING_H

// STL
#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace ufoexplorer
{
class TimerObject
{
 private:
	using Duration = std::chrono::duration<double, std::nano>;

 public:
	void start()
	{
		timing_ = true;
		time_ = std::chrono::high_resolution_clock::now();
	}

	void stop()
	{
		std::chrono::time_point<std::chrono::high_resolution_clock, Duration> now =
		    std::chrono::high_resolution_clock::now();
		last_ = Duration(now - time_).count();

		++samples_;

		double delta = last_ - mean_time_;
		mean_time_ += delta / samples_;
		double delta_2 = last_ - mean_time_;
		variance_time_ += delta * delta_2;

		total_time_ += last_;
		min_time_ = std::min(min_time_, last_);
		max_time_ = std::max(max_time_, last_);

		timing_ = false;
	}

	bool isTiming() const { return timing_; }

	std::optional<double> getCurrentTiming() const
	{
		if (!timing_) {
			return std::nullopt;
		}
		return Duration(std::chrono::high_resolution_clock::now() - time_).count();
	}

	double getLast() const { return last_; }

	std::size_t getNumSamples() const { return samples_; }

	double getTotal() const { return total_time_; }

	std::optional<double> getMin() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return min_time_;
	}

	std::optional<double> getMax() const
	{
		if (0 == samples_) {
			return std::nullopt;
		}
		return max_time_;
	}

	double getMean() const { return mean_time_; }

	std::optional<double> getVariance() const { return getSampleVariance(); }

	std::optional<double> getSampleVariance() const
	{
		if (2 > samples_) {
			return std::nullopt;
		}
		return variance_time_ / (samples_ - 1);
	}

	std::optional<double> getPopulationVariance() const
	{
		if (2 > samples_) {
			return std::nullopt;
		}
		return variance_time_ / (samples_);
	}

 private:
	std::chrono::time_point<std::chrono::high_resolution_clock, Duration> time_;

	bool timing_ = false;

	std::size_t samples_ = 0;
	double last_;
	double total_time_ = 0.0;
	double mean_time_ = 0.0;
	double variance_time_ = 0.0;
	double min_time_ = std::numeric_limits<double>::max();
	double max_time_ = std::numeric_limits<double>::lowest();

	// std::chrono::duration<double> last_;
	// std::chrono::duration<double> total_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> mean_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> variance_time_ = std::chrono::duration<double>::zero();
	// std::chrono::duration<double> min_time_ = std::chrono::duration<double>::max();
	// std::chrono::duration<double> max_time_ = std::chrono::duration<double>::min();
};

class Timing
{
 public:
	void start(std::string const& tag) { timers_[tag].start(); }

	void stop(std::string const& tag) { timers_.at(tag).stop(); }

	bool contains(std::string const& tag) const { return 0 < timers_.count(tag); }

	bool isTiming(std::string const& tag) const { return timers_.at(tag).isTiming(); }

	double getTotalWithCurrentSeconds(std::string const& tag) const
	{
		return getTotalSeconds(tag) + getCurrentTimingSeconds(tag);
	}

	double getTotalWithCurrentMilliseconds(std::string const& tag) const
	{
		return getTotalMilliseconds(tag) + getCurrentTimingMilliseconds(tag);
	}

	double getTotalWithCurrentMicroseconds(std::string const& tag) const
	{
		return getTotalMicroseconds(tag) + getCurrentTimingMicroseconds(tag);
	}

	double getCurrentTimingSeconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000000.0;
	}

	double getCurrentTimingMilliseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000000.0;
	}

	double getCurrentTimingMicroseconds(std::string const& tag) const
	{
		auto ct = timers_.at(tag).getCurrentTiming();
		if (!ct) {
			return -1;
		}
		return *ct / 1000.0;
	}

	double getLastSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000000000.0;
	}
	double getLastMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000000.0;
	}
	double getLastMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getLast() / 1000.0;
	}

	std::size_t getNumSamples(std::string const& tag) const
	{
		return timers_.at(tag).getNumSamples();
	}

	double getTotalSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000000000.0;
	}
	double getTotalMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000000.0;
	}
	double getTotalMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getTotal() / 1000.0;
	}

	double getMinSeconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000000.0;
	}
	double getMinMilliseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000000.0;
	}
	double getMinMicroseconds(std::string const& tag) const
	{
		auto min = timers_.at(tag).getMin();
		if (!min) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *min / 1000.0;
	}

	double getMaxSeconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000000.0;
	}
	double getMaxMilliseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000000.0;
	}
	double getMaxMicroseconds(std::string const& tag) const
	{
		auto max = timers_.at(tag).getMax();
		if (!max) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return *max / 1000.0;
	}

	double getMeanSeconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000000000.0;
	}
	double getMeanMilliseconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000000.0;
	}
	double getMeanMicroseconds(std::string const& tag) const
	{
		return timers_.at(tag).getMean() / 1000.0;
	}

	double getStdSeconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000000.0;
	}
	double getStdMilliseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000000.0;
	}
	double getStdMicroseconds(std::string const& tag) const
	{
		auto variance = timers_.at(tag).getVariance();
		if (!variance) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::sqrt(*variance) / 1000.0;
	}

	std::vector<std::string> getTags() const
	{
		std::vector<std::string> tags;
		tags.reserve(timers_.size());
		for (auto const& [tag, timer] : timers_) {
			tags.push_back(tag);
		}
		return tags;
	}

 private:
	std::map<std::string, TimerObject> timers_;
};
}  // namespace ufoexplorer

#endif  // UFOEXPLORER_TIMING_H
#ifndef STATISTICS_H
#define STATISTICS_H

#include <string>
#include <unordered_map>
#include <mutex>
#include <iostream>
#include <sstream>

class StatisticsImpl {
public:
	StatisticsImpl() = default;
	~StatisticsImpl() = default;


	void AddSample(const std::string& tag, const double sample) {
		mtx_.lock();
		tag_to_samples_[tag].push_back(sample);
		mtx_.unlock();
	}

	void Reset(){
		mtx_.lock();
		tag_to_samples_.clear();
		mtx_.unlock();
	}

	void PrintStatistics() {
		mtx_.lock();
		std::stringstream os;
		os << "---------------------------\n";
		os << "Statiscs\n";
		os << "---------------------------\n";
		os << "Tag\tSample Number\t\n";
		for (const auto& [tag, samples] : tag_to_samples_) {
			os << tag << "\t" << samples.size() << "\n";
		}
		std::cout << os.str() << std::endl;
		mtx_.unlock();
	}


private:
	std::unordered_map<std::string, std::vector<double>> tag_to_samples_;
	std::mutex mtx_;
};


class StatisticsCollector {
public:

	static void AddSample(const std::string& tag, const double sample) {
		Instance().AddSample(tag, sample);
	}

	static void Reset() {
		Instance().Reset();
	}

	static void PrintStatistics() {
		Instance().PrintStatistics();
	}

	static StatisticsImpl& Instance() {
		static StatisticsImpl statistics;
		return statistics;
	}

private:
	StatisticsCollector();
	~StatisticsCollector();
};

#endif
#ifndef STATISTICS_H
#define STATISTICS_H

#include <string>
#include <unordered_map>
#include <mutex>
#include <iostream>
#include <sstream>
#include<Eigen/Dense>
#include <fstream>
#include <iomanip>      // std::setprecision


namespace statistics {
struct Trajectory {
	std::vector<Eigen::Quaterniond> quaternions;
	std::vector<Eigen::Vector3d> trans;
	std::vector<Eigen::Vector3d> vels;
	std::vector<double> timestamp_secs;
};

class StatisticsImpl {
public:
	StatisticsImpl() = default;
	~StatisticsImpl() = default;


	void AddSample(const std::string& tag, const double sample) {
		mtx_.lock();
		tag_to_samples_[tag].push_back(sample);
		mtx_.unlock();
	}

	void AddInitializationTrajectory(const Trajectory& initialization_traj) {
		initialization_trajectories_.push_back(initialization_traj);
	}

	void AddTrackingTrajectory(const Trajectory& tacking_traj) {
		tracking_trajectories_.push_back(tacking_traj);
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

		os << "Initialization Trajectory Number:\t" << initialization_trajectories_.size() << "\n";
		os << "Tracking Trajectory Number:\t" << tracking_trajectories_.size() << "\n";
		std::cout << os.str() << std::endl;
		mtx_.unlock();
	}

	void SaveInitializationTrajectory(const std::string& csv_path) {
		std::cout << "Saving Initialization Trajectory: " << csv_path << std::endl;
		mtx_.lock();
		SaveTrajectories(initialization_trajectories_, csv_path);
		mtx_.unlock();
		std::cout << "End saving Initialization Trajectory: " << csv_path << std::endl;
	}

	void SaveTrackingTrajectory(const std::string& csv_path) {
		std::cout << "Saving Tracking Trajectory: " << csv_path << std::endl;
		mtx_.lock();
		SaveTrajectories(tracking_trajectories_, csv_path);
		mtx_.unlock();
		std::cout << "End saving Tracking Trajectory: " << csv_path << std::endl;
	}

private:
	void SaveTrajectories(const std::vector<Trajectory>& trajectories, const std::string& csv_path){
		std::ofstream f;
		f.open(csv_path.c_str());
    	assert(f.isOpened());
    	f << std::fixed;
    	f << "trajectory_id timestamp_ns q_x q_y q_z q_w x y z v_x v_y v_z" << std::endl;
    	int trajectory_id = 0;
		for (const auto& traj : trajectories) {
			const int num_kfs = traj.timestamp_secs.size();
			for (int i = 0; i < num_kfs; i++) {
				const double timestamp_sec = traj.timestamp_secs[i];
				const Eigen::Quaterniond& quaternion = traj.quaternions[i];
				const Eigen::Vector3d& translation = traj.trans[i];
				const Eigen::Vector3d& velocity = traj.vels[i];
				f << trajectory_id << " " << std::setprecision(6) << 1e9*timestamp_sec
				  << " " <<  std::setprecision(9) << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
				  << " " << translation[0] << " " << translation[1] << " " << translation[2] 
				  << " " << velocity[0] << " " << velocity[1] << " " << velocity[2] << std::endl;
			}
			trajectory_id++;
		}
		f.close();
	}

	std::unordered_map<std::string, std::vector<double>> tag_to_samples_;
	std::vector<Trajectory> initialization_trajectories_;
	std::vector<Trajectory> tracking_trajectories_;
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

	static void AddInitializationTrajectory(const Trajectory& initialization_traj) {
		Instance().AddInitializationTrajectory(initialization_traj);
	}

	static void AddTrackingTrajectory(const Trajectory& initialization_traj) {
		Instance().AddTrackingTrajectory(initialization_traj);
	}

	static void SaveInitializationTrajectory(const std::string& csv_path) {
		Instance().SaveInitializationTrajectory(csv_path);
	}

	static void SaveTrackingTrajectory(const std::string& csv_path) {
		Instance().SaveTrackingTrajectory(csv_path);
	}

	static StatisticsImpl& Instance() {
		static StatisticsImpl statistics;
		return statistics;
	}

private:
	StatisticsCollector();
	~StatisticsCollector();
};

}
#endif
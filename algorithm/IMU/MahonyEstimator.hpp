#ifndef MAHONY_ESTIMATOR_HPP
#define MAHONY_ESTIMATOR_HPP

#include "PoseEstimator.hpp"

class MahonyEstimator : public PoseEstimator
{
private :
	MahonyEstimator(){;}
public:
	static MahonyEstimator* Instance()
	{
		static MahonyEstimator instance;
		return &instance;
	}
    virtual void Init();
    virtual void Update();
};

#endif

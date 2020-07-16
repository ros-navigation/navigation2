#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

#include <interfaces/matcher_base.hpp>
#include <interfaces/motion_model_base.hpp>

namespace nav2_localization_base
{
	class Solver2d
	{
		public:
			// From nav2 localization interfaces
			Matcher2d matcher;
			MotionModel motion_model;

		protected:
			Solver2d(){}
	};
};

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_

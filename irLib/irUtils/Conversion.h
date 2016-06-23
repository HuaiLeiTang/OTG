#pragma once
#include <Eigen\Dense>

namespace irLib
{
	namespace irUtils
	{
		template <typename DestClass, typename EigenDerived>
		DestClass eigen2osgVec(const Eigen::DenseBase<EigenDerived>& input)
		{
			DestClass ret;
			for (int i = 0; i < input.size(); i++)
				ret[i] = (float)input[i];
			return ret;
		}

		template <typename DestClass, typename EigenDerived>
		DestClass eigen2osgMat(const Eigen::DenseBase<EigenDerived>& input)
		{
			DestClass ret;
			for (int j = 0; j < input.cols(); j++)
				for (int i = 0; i < input.rows(); i++)
					ret(i, j) = input(i, j);
			return ret;
		}
	}
}
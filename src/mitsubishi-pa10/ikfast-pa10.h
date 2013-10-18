#ifndef _IKFAST_PA10_H
#define _IKFAST_PA10_H

#include "ikfast.h"

typedef double IkReal;

namespace IKFAST_6DF51{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_6DF41{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_6DF3{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_6DF21{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_6DF11{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_6DF01{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}

#endif

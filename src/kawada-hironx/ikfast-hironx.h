#ifndef _IKFAST_HIRONX_H
#define _IKFAST_HIRONX_H

#include "ikfast.h"

typedef double IkReal;

namespace IKFAST_LOOKAT3D{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_LOOKAT3DF0{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_RIGHT6D{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_RIGHT6DF0{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_LEFT6D{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_LEFT6DF0{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_RIGHT5DF8{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}
namespace IKFAST_LEFT5DF18{
	int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint);
}

#endif // _IKFAST_HIRONX_H
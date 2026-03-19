#include "thunder_franka.h"
#include "franka_gen.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// --------------------------- //
// ----- Python bindings ----- //
// --------------------------- //
namespace py = pybind11;
PYBIND11_MODULE(thunder_franka_py, m) {
	py::class_<thunder_franka>(m, "thunder_franka")
		.def(py::init<>())

		.def_readonly("Dl_order", &thunder_franka::Dl_order, "Order of the link friction model")
		.def_readonly("STD_PAR_LINK", &thunder_franka::STD_PAR_LINK, "Standard number of dynamic parameters per link")
		.def_readonly("jointsType", &thunder_franka::jointsType, "Type of joints")
		.def_readonly("numJoints", &thunder_franka::numJoints, "Number of joints")
		.def_readonly("r1", &thunder_franka::r1, "Index of first revolute joint")
		.def_readonly("r2", &thunder_franka::r2, "Index of sequent revolute joint not aligned with r1")

		.def("get_d3q", &thunder_franka::get_d3q, "Get parameter: d3q")
		.def("get_d4q", &thunder_franka::get_d4q, "Get parameter: d4q")
		.def("get_ddq", &thunder_franka::get_ddq, "Get parameter: ddq")
		.def("get_ddqr", &thunder_franka::get_ddqr, "Get parameter: ddqr")
		.def("get_dq", &thunder_franka::get_dq, "Get parameter: dq")
		.def("get_dqr", &thunder_franka::get_dqr, "Get parameter: dqr")
		.def("get_par_DYN", &thunder_franka::get_par_DYN, "Get parameter: par_DYN")
		.def("get_par_Dl", &thunder_franka::get_par_Dl, "Get parameter: par_Dl")
		.def("get_par_Ln2EE", &thunder_franka::get_par_Ln2EE, "Get parameter: par_Ln2EE")
		.def("get_par_REG", &thunder_franka::get_par_REG, "Get parameter: par_REG")
		.def("get_par_REG_red", &thunder_franka::get_par_REG_red, "Get parameter: par_REG_red")
		.def("get_q", &thunder_franka::get_q, "Get parameter: q")
		.def("get_w", &thunder_franka::get_w, "Get parameter: w")
		.def("set_d3q", &thunder_franka::set_d3q, "Set parameter: d3q", py::arg("value"))
		.def("set_d4q", &thunder_franka::set_d4q, "Set parameter: d4q", py::arg("value"))
		.def("set_ddq", &thunder_franka::set_ddq, "Set parameter: ddq", py::arg("value"))
		.def("set_ddqr", &thunder_franka::set_ddqr, "Set parameter: ddqr", py::arg("value"))
		.def("set_dq", &thunder_franka::set_dq, "Set parameter: dq", py::arg("value"))
		.def("set_dqr", &thunder_franka::set_dqr, "Set parameter: dqr", py::arg("value"))
		.def("set_par_DYN", &thunder_franka::set_par_DYN, "Set parameter: par_DYN", py::arg("value"))
		.def("set_par_Dl", &thunder_franka::set_par_Dl, "Set parameter: par_Dl", py::arg("value"))
		.def("set_par_Ln2EE", &thunder_franka::set_par_Ln2EE, "Set parameter: par_Ln2EE", py::arg("value"))
		.def("set_par_REG", &thunder_franka::set_par_REG, "Set parameter: par_REG", py::arg("value"))
		.def("set_par_REG_red", &thunder_franka::set_par_REG_red, "Set parameter: par_REG_red", py::arg("value"))
		.def("set_q", &thunder_franka::set_q, "Set parameter: q", py::arg("value"))
		.def("set_w", &thunder_franka::set_w, "Set parameter: w", py::arg("value"))

		.def("get_C", &thunder_franka::get_C, "Manipulator Coriolis matrix")
		.def("get_C_ddot", &thunder_franka::get_C_ddot, "Second time derivative of the Coriolis matrix")
		.def("get_C_dot", &thunder_franka::get_C_dot, "Time derivative of the Coriolis matrix")
		.def("get_C_std", &thunder_franka::get_C_std, "Classic formulation of the manipulator Coriolis matrix")
		.def("get_Dl1", &thunder_franka::get_Dl1, "SEA manipulator link damping, order 1")
		.def("get_Dl2", &thunder_franka::get_Dl2, "SEA manipulator link damping, order 2")
		.def("get_G", &thunder_franka::get_G, "Manipulator gravity terms")
		.def("get_G_ddot", &thunder_franka::get_G_ddot, "Second time derivative of the gravity vector")
		.def("get_G_dot", &thunder_franka::get_G_dot, "Time derivative of the gravity vector")
		.def("get_J_1", &thunder_franka::get_J_1, "Jacobian of frame 1")
		.def("get_J_2", &thunder_franka::get_J_2, "Jacobian of frame 2")
		.def("get_J_3", &thunder_franka::get_J_3, "Jacobian of frame 3")
		.def("get_J_4", &thunder_franka::get_J_4, "Jacobian of frame 4")
		.def("get_J_5", &thunder_franka::get_J_5, "Jacobian of frame 5")
		.def("get_J_6", &thunder_franka::get_J_6, "Jacobian of frame 6")
		.def("get_J_7", &thunder_franka::get_J_7, "Jacobian of frame 7")
		.def("get_J_8", &thunder_franka::get_J_8, "Jacobian of frame 8")
		.def("get_J_cm_1", &thunder_franka::get_J_cm_1, "Jacobian of center of mass of link 1")
		.def("get_J_cm_2", &thunder_franka::get_J_cm_2, "Jacobian of center of mass of link 2")
		.def("get_J_cm_3", &thunder_franka::get_J_cm_3, "Jacobian of center of mass of link 3")
		.def("get_J_cm_4", &thunder_franka::get_J_cm_4, "Jacobian of center of mass of link 4")
		.def("get_J_cm_5", &thunder_franka::get_J_cm_5, "Jacobian of center of mass of link 5")
		.def("get_J_cm_6", &thunder_franka::get_J_cm_6, "Jacobian of center of mass of link 6")
		.def("get_J_cm_7", &thunder_franka::get_J_cm_7, "Jacobian of center of mass of link 7")
		.def("get_J_ee", &thunder_franka::get_J_ee, "Jacobian of the end-effector")
		.def("get_J_ee_ddot", &thunder_franka::get_J_ee_ddot, "Time second derivative of jacobian matrix")
		.def("get_J_ee_dot", &thunder_franka::get_J_ee_dot, "Time derivative of jacobian matrix")
		.def("get_J_ee_pinv", &thunder_franka::get_J_ee_pinv, "Pseudo-Inverse of jacobian matrix")
		.def("get_M", &thunder_franka::get_M, "Manipulator mass matrix")
		.def("get_M_ddot", &thunder_franka::get_M_ddot, "Second time derivative of the mass matrix")
		.def("get_M_dot", &thunder_franka::get_M_dot, "Time derivative of the mass matrix")
		.def("get_T_0", &thunder_franka::get_T_0, "relative transformation from frame world to base")
		.def("get_T_1", &thunder_franka::get_T_1, "relative transformation from frame0to frame 1")
		.def("get_T_2", &thunder_franka::get_T_2, "relative transformation from frame1to frame 2")
		.def("get_T_3", &thunder_franka::get_T_3, "relative transformation from frame2to frame 3")
		.def("get_T_4", &thunder_franka::get_T_4, "relative transformation from frame3to frame 4")
		.def("get_T_5", &thunder_franka::get_T_5, "relative transformation from frame4to frame 5")
		.def("get_T_6", &thunder_franka::get_T_6, "relative transformation from frame5to frame 6")
		.def("get_T_7", &thunder_franka::get_T_7, "relative transformation from frame6to frame 7")
		.def("get_T_JOINT_P", &thunder_franka::get_T_JOINT_P, "Template transformation of general prismatic joint R")
		.def("get_T_JOINT_R", &thunder_franka::get_T_JOINT_R, "Template transformation of general rotoidal joint R")
		.def("get_T_w_0", &thunder_franka::get_T_w_0, "absolute transformation from frame world to base")
		.def("get_T_w_1", &thunder_franka::get_T_w_1, "absolute transformation from frame base to frame 1")
		.def("get_T_w_2", &thunder_franka::get_T_w_2, "absolute transformation from frame base to frame 2")
		.def("get_T_w_3", &thunder_franka::get_T_w_3, "absolute transformation from frame base to frame 3")
		.def("get_T_w_4", &thunder_franka::get_T_w_4, "absolute transformation from frame base to frame 4")
		.def("get_T_w_5", &thunder_franka::get_T_w_5, "absolute transformation from frame base to frame 5")
		.def("get_T_w_6", &thunder_franka::get_T_w_6, "absolute transformation from frame base to frame 6")
		.def("get_T_w_7", &thunder_franka::get_T_w_7, "absolute transformation from frame base to frame 7")
		.def("get_T_w_8", &thunder_franka::get_T_w_8, "absolute transformation from frame base to end_effector")
		.def("get_T_w_ee", &thunder_franka::get_T_w_ee, "absolute transformation from frame 0 to end_effector")
		.def("get_Yr", &thunder_franka::get_Yr, "Manipulator regressor matrix")
		.def("get_Yr_red", &thunder_franka::get_Yr_red, "Regressor defined w.r.t the set of base iniertial parameters.")
		.def("get_beta", &thunder_franka::get_beta, "linear relationship between full dyn parameters and the reduced set. beta s.t. par_red = beta*par.")
		.def("get_dl", &thunder_franka::get_dl, "Manipulator link friction")
		.def("get_dyn2reg", &thunder_franka::get_dyn2reg, "Conversion from dynamic to regressor parameters")
		.def("get_par_KIN", &thunder_franka::get_par_KIN, "Internal kinematic parameters.")
		.def("get_reg2dyn", &thunder_franka::get_reg2dyn, "Conversion from regressor to dynamic parameters")
		.def("get_reg2red", &thunder_franka::get_reg2red, "Conversion from regressor to base inertial parameters.")
		.def("get_reg_C", &thunder_franka::get_reg_C, "Regressor matrix of term C*dqr")
		.def("get_reg_C_red", &thunder_franka::get_reg_C_red, "Regressor of centripetal defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_G", &thunder_franka::get_reg_G, "Regressor matrix of term G")
		.def("get_reg_G_red", &thunder_franka::get_reg_G_red, "Regressor of gravity defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_JTw", &thunder_franka::get_reg_JTw, "Regressor matrix of the quantity J^T*w")
		.def("get_reg_Jdq", &thunder_franka::get_reg_Jdq, "Regressor matrix of the quantity J*dq")
		.def("get_reg_M", &thunder_franka::get_reg_M, "Regressor matrix of term M*ddqr")
		.def("get_reg_M_red", &thunder_franka::get_reg_M_red, "Regressor of masses defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_dl", &thunder_franka::get_reg_dl, "Regressor matrix of the link friction");
}

thunder_franka::thunder_franka(){}

// get the parameter: d3q
Vector<double,7> thunder_franka::get_d3q() {return d3q;}
// get the parameter: d4q
Vector<double,7> thunder_franka::get_d4q() {return d4q;}
// get the parameter: ddq
Vector<double,7> thunder_franka::get_ddq() {return ddq;}
// get the parameter: ddqr
Vector<double,7> thunder_franka::get_ddqr() {return ddqr;}
// get the parameter: dq
Vector<double,7> thunder_franka::get_dq() {return dq;}
// get the parameter: dqr
Vector<double,7> thunder_franka::get_dqr() {return dqr;}
// get the parameter: par_DYN
Vector<double,70> thunder_franka::get_par_DYN() {return par_DYN;}
// get the parameter: par_Dl
Vector<double,14> thunder_franka::get_par_Dl() {return par_Dl;}
// get the parameter: par_Ln2EE
Vector<double,6> thunder_franka::get_par_Ln2EE() {return par_Ln2EE;}
// get the parameter: par_REG
Vector<double,70> thunder_franka::get_par_REG() {return par_REG;}
// get the parameter: par_REG_red
Vector<double,43> thunder_franka::get_par_REG_red() {return par_REG_red;}
// get the parameter: q
Vector<double,7> thunder_franka::get_q() {return q;}
// get the parameter: w
Vector<double,6> thunder_franka::get_w() {return w;}
// set the parameter: d3q
void thunder_franka::set_d3q(Vector<double,7> value) {d3q = value;}
// set the parameter: d4q
void thunder_franka::set_d4q(Vector<double,7> value) {d4q = value;}
// set the parameter: ddq
void thunder_franka::set_ddq(Vector<double,7> value) {ddq = value;}
// set the parameter: ddqr
void thunder_franka::set_ddqr(Vector<double,7> value) {ddqr = value;}
// set the parameter: dq
void thunder_franka::set_dq(Vector<double,7> value) {dq = value;}
// set the parameter: dqr
void thunder_franka::set_dqr(Vector<double,7> value) {dqr = value;}
// set the parameter: par_DYN
void thunder_franka::set_par_DYN(Vector<double,70> value) {par_DYN = value;}
// set the parameter: par_Dl
void thunder_franka::set_par_Dl(Vector<double,14> value) {par_Dl = value;}
// set the parameter: par_Ln2EE
void thunder_franka::set_par_Ln2EE(Vector<double,6> value) {par_Ln2EE = value;}
// set the parameter: par_REG
void thunder_franka::set_par_REG(Vector<double,70> value) {par_REG = value;}
// set the parameter: par_REG_red
void thunder_franka::set_par_REG_red(Vector<double,43> value) {par_REG_red = value;}
// set the parameter: q
void thunder_franka::set_q(Vector<double,7> value) {q = value;}
// set the parameter: w
void thunder_franka::set_w(Vector<double,6> value) {w = value;}

// Save parameters to file
int thunder_franka::save_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		vector<double> d3q_vect(d3q.data(), d3q.data() + 7);
		yamlFile["d3q"] = d3q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		vector<double> d4q_vect(d4q.data(), d4q.data() + 7);
		yamlFile["d4q"] = d4q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		vector<double> ddq_vect(ddq.data(), ddq.data() + 7);
		yamlFile["ddq"] = ddq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		vector<double> ddqr_vect(ddqr.data(), ddqr.data() + 7);
		yamlFile["ddqr"] = ddqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		vector<double> dq_vect(dq.data(), dq.data() + 7);
		yamlFile["dq"] = dq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		vector<double> dqr_vect(dqr.data(), dqr.data() + 7);
		yamlFile["dqr"] = dqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		vector<double> par_DYN_vect(par_DYN.data(), par_DYN.data() + 70);
		yamlFile["par_DYN"] = par_DYN_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Dl"))){
		vector<double> par_Dl_vect(par_Dl.data(), par_Dl.data() + 14);
		yamlFile["par_Dl"] = par_Dl_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ln2EE"))){
		vector<double> par_Ln2EE_vect(par_Ln2EE.data(), par_Ln2EE.data() + 6);
		yamlFile["par_Ln2EE"] = par_Ln2EE_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		vector<double> par_REG_vect(par_REG.data(), par_REG.data() + 70);
		yamlFile["par_REG"] = par_REG_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG_red"))){
		vector<double> par_REG_red_vect(par_REG_red.data(), par_REG_red.data() + 43);
		yamlFile["par_REG_red"] = par_REG_red_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		vector<double> q_vect(q.data(), q.data() + 7);
		yamlFile["q"] = q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		vector<double> w_vect(w.data(), w.data() + 6);
		yamlFile["w"] = w_vect;
	}

	try {
		YAML::Emitter emitter;
		emitter.SetIndent(2);
		emitter.SetSeqFormat(YAML::Flow);
		emitter << yamlFile << YAML::Newline;
		std::ofstream fout(par_file);
		fout << emitter.c_str();
		fout.close();
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		return 0;
	}
	return 1;
}


// Load parameters from file
int thunder_franka::load_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;
	try {
		yamlFile = YAML::LoadFile(par_file);

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while loading parameters: " << e.what() << std::endl;
		return 0;
	}

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		if (yamlFile["d3q"]){
			vector<double> vec(yamlFile["d3q"].as<vector<double>>());
			d3q = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d3q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		if (yamlFile["d4q"]){
			vector<double> vec(yamlFile["d4q"].as<vector<double>>());
			d4q = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d4q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		if (yamlFile["ddq"]){
			vector<double> vec(yamlFile["ddq"].as<vector<double>>());
			ddq = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		if (yamlFile["ddqr"]){
			vector<double> vec(yamlFile["ddqr"].as<vector<double>>());
			ddqr = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		if (yamlFile["dq"]){
			vector<double> vec(yamlFile["dq"].as<vector<double>>());
			dq = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		if (yamlFile["dqr"]){
			vector<double> vec(yamlFile["dqr"].as<vector<double>>());
			dqr = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		if (yamlFile["par_DYN"]){
			vector<double> vec(yamlFile["par_DYN"].as<vector<double>>());
			par_DYN = Eigen::Map<Vector<double,70>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_DYN not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Dl"))){
		if (yamlFile["par_Dl"]){
			vector<double> vec(yamlFile["par_Dl"].as<vector<double>>());
			par_Dl = Eigen::Map<Vector<double,14>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_Dl not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ln2EE"))){
		if (yamlFile["par_Ln2EE"]){
			vector<double> vec(yamlFile["par_Ln2EE"].as<vector<double>>());
			par_Ln2EE = Eigen::Map<Vector<double,6>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_Ln2EE not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		if (yamlFile["par_REG"]){
			vector<double> vec(yamlFile["par_REG"].as<vector<double>>());
			par_REG = Eigen::Map<Vector<double,70>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG_red"))){
		if (yamlFile["par_REG_red"]){
			vector<double> vec(yamlFile["par_REG_red"].as<vector<double>>());
			par_REG_red = Eigen::Map<Vector<double,43>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG_red not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		if (yamlFile["q"]){
			vector<double> vec(yamlFile["q"].as<vector<double>>());
			q = Eigen::Map<Vector<double,7>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		if (yamlFile["w"]){
			vector<double> vec(yamlFile["w"].as<vector<double>>());
			w = Eigen::Map<Vector<double,6>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: w not found!" << std::endl;
			return 0;
		}
	}

	return 1;
}


// Manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_fun_SZ_IW];
	thread_local double p4[franka_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_ddot_fun_SZ_IW];
	thread_local double p4[franka_C_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), d3q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_dot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_dot_fun_SZ_IW];
	thread_local double p4[franka_C_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Classic formulation of the manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_std() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_std_fun_SZ_IW];
	thread_local double p4[franka_C_std_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_std_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// SEA manipulator link damping, order 1
Eigen::Matrix<double,7,7> thunder_franka::get_Dl1() {
	thread_local double buffer[49];
	thread_local long long p3[franka_Dl1_fun_SZ_IW];
	thread_local double p4[franka_Dl1_fun_SZ_W];
	const double* input_[] = {par_Dl.data()};
	double* output_[] = {buffer};
	int check = franka_Dl1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// SEA manipulator link damping, order 2
Eigen::Matrix<double,7,7> thunder_franka::get_Dl2() {
	thread_local double buffer[49];
	thread_local long long p3[franka_Dl2_fun_SZ_IW];
	thread_local double p4[franka_Dl2_fun_SZ_W];
	const double* input_[] = {par_Dl.data()};
	double* output_[] = {buffer};
	int check = franka_Dl2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Manipulator gravity terms
Eigen::Matrix<double,7,1> thunder_franka::get_G() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_fun_SZ_IW];
	thread_local double p4[franka_G_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Second time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_franka::get_G_ddot() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_ddot_fun_SZ_IW];
	thread_local double p4[franka_G_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_franka::get_G_dot() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_dot_fun_SZ_IW];
	thread_local double p4[franka_G_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Jacobian of frame 1
Eigen::Matrix<double,6,7> thunder_franka::get_J_1() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_1_fun_SZ_IW];
	thread_local double p4[franka_J_1_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 2
Eigen::Matrix<double,6,7> thunder_franka::get_J_2() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_2_fun_SZ_IW];
	thread_local double p4[franka_J_2_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 3
Eigen::Matrix<double,6,7> thunder_franka::get_J_3() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_3_fun_SZ_IW];
	thread_local double p4[franka_J_3_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 4
Eigen::Matrix<double,6,7> thunder_franka::get_J_4() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_4_fun_SZ_IW];
	thread_local double p4[franka_J_4_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 5
Eigen::Matrix<double,6,7> thunder_franka::get_J_5() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_5_fun_SZ_IW];
	thread_local double p4[franka_J_5_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 6
Eigen::Matrix<double,6,7> thunder_franka::get_J_6() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_6_fun_SZ_IW];
	thread_local double p4[franka_J_6_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 7
Eigen::Matrix<double,6,7> thunder_franka::get_J_7() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_7_fun_SZ_IW];
	thread_local double p4[franka_J_7_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_J_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 8
Eigen::Matrix<double,6,7> thunder_franka::get_J_8() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_8_fun_SZ_IW];
	thread_local double p4[franka_J_8_fun_SZ_W];
	const double* input_[] = {q.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_J_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 1
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_1() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_1_fun_SZ_IW];
	thread_local double p4[franka_J_cm_1_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 2
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_2() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_2_fun_SZ_IW];
	thread_local double p4[franka_J_cm_2_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 3
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_3() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_3_fun_SZ_IW];
	thread_local double p4[franka_J_cm_3_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 4
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_4() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_4_fun_SZ_IW];
	thread_local double p4[franka_J_cm_4_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 5
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_5() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_5_fun_SZ_IW];
	thread_local double p4[franka_J_cm_5_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 6
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_6() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_6_fun_SZ_IW];
	thread_local double p4[franka_J_cm_6_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 7
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_7() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_7_fun_SZ_IW];
	thread_local double p4[franka_J_cm_7_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of the end-effector
Eigen::Matrix<double,6,7> thunder_franka::get_J_ee() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_ee_fun_SZ_IW];
	thread_local double p4[franka_J_ee_fun_SZ_W];
	const double* input_[] = {q.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_J_ee_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time second derivative of jacobian matrix
Eigen::Matrix<double,6,7> thunder_franka::get_J_ee_ddot() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_ee_ddot_fun_SZ_IW];
	thread_local double p4[franka_J_ee_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_J_ee_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time derivative of jacobian matrix
Eigen::Matrix<double,6,7> thunder_franka::get_J_ee_dot() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_ee_dot_fun_SZ_IW];
	thread_local double p4[franka_J_ee_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_J_ee_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Pseudo-Inverse of jacobian matrix
Eigen::Matrix<double,7,6> thunder_franka::get_J_ee_pinv() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_ee_pinv_fun_SZ_IW];
	thread_local double p4[franka_J_ee_pinv_fun_SZ_W];
	const double* input_[] = {q.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_J_ee_pinv_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,6>>(buffer);
}

// Manipulator mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_fun_SZ_IW];
	thread_local double p4[franka_M_fun_SZ_W];
	const double* input_[] = {q.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_ddot_fun_SZ_IW];
	thread_local double p4[franka_M_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M_dot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_dot_fun_SZ_IW];
	thread_local double p4[franka_M_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// relative transformation from frame world to base
Eigen::Matrix<double,4,4> thunder_franka::get_T_0() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_0_fun_SZ_IW];
	thread_local double p4[franka_T_0_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = franka_T_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame0to frame 1
Eigen::Matrix<double,4,4> thunder_franka::get_T_1() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_1_fun_SZ_IW];
	thread_local double p4[franka_T_1_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame1to frame 2
Eigen::Matrix<double,4,4> thunder_franka::get_T_2() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_2_fun_SZ_IW];
	thread_local double p4[franka_T_2_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame2to frame 3
Eigen::Matrix<double,4,4> thunder_franka::get_T_3() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_3_fun_SZ_IW];
	thread_local double p4[franka_T_3_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame3to frame 4
Eigen::Matrix<double,4,4> thunder_franka::get_T_4() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_4_fun_SZ_IW];
	thread_local double p4[franka_T_4_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame4to frame 5
Eigen::Matrix<double,4,4> thunder_franka::get_T_5() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_5_fun_SZ_IW];
	thread_local double p4[franka_T_5_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame5to frame 6
Eigen::Matrix<double,4,4> thunder_franka::get_T_6() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_6_fun_SZ_IW];
	thread_local double p4[franka_T_6_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame6to frame 7
Eigen::Matrix<double,4,4> thunder_franka::get_T_7() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_7_fun_SZ_IW];
	thread_local double p4[franka_T_7_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general prismatic joint R
Eigen::Matrix<double,4,4> thunder_franka::get_T_JOINT_P(Vector<double,1> q_joint, Vector<double,3> axes) {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_JOINT_P_fun_SZ_IW];
	thread_local double p4[franka_T_JOINT_P_fun_SZ_W];
	const double* input_[] = {q_joint.data()};
	double* output_[] = {buffer};
	int check = franka_T_JOINT_P_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general rotoidal joint R
Eigen::Matrix<double,4,4> thunder_franka::get_T_JOINT_R(Vector<double,1> q_joint, Vector<double,3> axes) {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_JOINT_R_fun_SZ_IW];
	thread_local double p4[franka_T_JOINT_R_fun_SZ_W];
	const double* input_[] = {q_joint.data()};
	double* output_[] = {buffer};
	int check = franka_T_JOINT_R_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to base
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_0() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_0_fun_SZ_IW];
	thread_local double p4[franka_T_w_0_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = franka_T_w_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 1
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_1() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_1_fun_SZ_IW];
	thread_local double p4[franka_T_w_1_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 2
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_2() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_2_fun_SZ_IW];
	thread_local double p4[franka_T_w_2_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 3
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_3() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_3_fun_SZ_IW];
	thread_local double p4[franka_T_w_3_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 4
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_4() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_4_fun_SZ_IW];
	thread_local double p4[franka_T_w_4_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 5
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_5() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_5_fun_SZ_IW];
	thread_local double p4[franka_T_w_5_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 6
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_6() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_6_fun_SZ_IW];
	thread_local double p4[franka_T_w_6_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 7
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_7() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_7_fun_SZ_IW];
	thread_local double p4[franka_T_w_7_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to end_effector
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_8() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_8_fun_SZ_IW];
	thread_local double p4[franka_T_w_8_fun_SZ_W];
	const double* input_[] = {q.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame 0 to end_effector
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_ee() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_ee_fun_SZ_IW];
	thread_local double p4[franka_T_w_ee_fun_SZ_W];
	const double* input_[] = {q.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_ee_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Manipulator regressor matrix
Eigen::Matrix<double,7,70> thunder_franka::get_Yr() {
	thread_local double buffer[490];
	thread_local long long p3[franka_Yr_fun_SZ_IW];
	thread_local double p4[franka_Yr_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
	double* output_[] = {buffer};
	int check = franka_Yr_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,70>>(buffer);
}

// Regressor defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,7,43> thunder_franka::get_Yr_red() {
	thread_local double buffer[301];
	thread_local long long p3[franka_Yr_red_fun_SZ_IW];
	thread_local double p4[franka_Yr_red_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
	double* output_[] = {buffer};
	int check = franka_Yr_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,43>>(buffer);
}

// linear relationship between full dyn parameters and the reduced set. beta s.t. par_red = beta*par.
Eigen::Matrix<double,43,70> thunder_franka::get_beta() {
	thread_local double buffer[3010];
	thread_local long long p3[franka_beta_fun_SZ_IW];
	thread_local double p4[franka_beta_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = franka_beta_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,43,70>>(buffer);
}

// Manipulator link friction
Eigen::Matrix<double,7,1> thunder_franka::get_dl() {
	thread_local double buffer[7];
	thread_local long long p3[franka_dl_fun_SZ_IW];
	thread_local double p4[franka_dl_fun_SZ_W];
	const double* input_[] = {dq.data(), par_Dl.data()};
	double* output_[] = {buffer};
	int check = franka_dl_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Conversion from dynamic to regressor parameters
Eigen::Matrix<double,70,1> thunder_franka::get_dyn2reg() {
	thread_local double buffer[70];
	thread_local long long p3[franka_dyn2reg_fun_SZ_IW];
	thread_local double p4[franka_dyn2reg_fun_SZ_W];
	const double* input_[] = {par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_dyn2reg_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,70,1>>(buffer);
}

// Internal kinematic parameters.
Eigen::Matrix<double,42,1> thunder_franka::get_par_KIN() {
	thread_local double buffer[42];
	thread_local long long p3[franka_par_KIN_fun_SZ_IW];
	thread_local double p4[franka_par_KIN_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = franka_par_KIN_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,42,1>>(buffer);
}

// Conversion from regressor to dynamic parameters
Eigen::Matrix<double,70,1> thunder_franka::get_reg2dyn() {
	thread_local double buffer[70];
	thread_local long long p3[franka_reg2dyn_fun_SZ_IW];
	thread_local double p4[franka_reg2dyn_fun_SZ_W];
	const double* input_[] = {par_REG.data()};
	double* output_[] = {buffer};
	int check = franka_reg2dyn_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,70,1>>(buffer);
}

// Conversion from regressor to base inertial parameters.
Eigen::Matrix<double,43,1> thunder_franka::get_reg2red() {
	thread_local double buffer[43];
	thread_local long long p3[franka_reg2red_fun_SZ_IW];
	thread_local double p4[franka_reg2red_fun_SZ_W];
	const double* input_[] = {par_REG.data()};
	double* output_[] = {buffer};
	int check = franka_reg2red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,43,1>>(buffer);
}

// Regressor matrix of term C*dqr
Eigen::Matrix<double,7,70> thunder_franka::get_reg_C() {
	thread_local double buffer[490];
	thread_local long long p3[franka_reg_C_fun_SZ_IW];
	thread_local double p4[franka_reg_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data()};
	double* output_[] = {buffer};
	int check = franka_reg_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,70>>(buffer);
}

// Regressor of centripetal defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,7,43> thunder_franka::get_reg_C_red() {
	thread_local double buffer[301];
	thread_local long long p3[franka_reg_C_red_fun_SZ_IW];
	thread_local double p4[franka_reg_C_red_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data()};
	double* output_[] = {buffer};
	int check = franka_reg_C_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,43>>(buffer);
}

// Regressor matrix of term G
Eigen::Matrix<double,7,70> thunder_franka::get_reg_G() {
	thread_local double buffer[490];
	thread_local long long p3[franka_reg_G_fun_SZ_IW];
	thread_local double p4[franka_reg_G_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_reg_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,70>>(buffer);
}

// Regressor of gravity defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,7,43> thunder_franka::get_reg_G_red() {
	thread_local double buffer[301];
	thread_local long long p3[franka_reg_G_red_fun_SZ_IW];
	thread_local double p4[franka_reg_G_red_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = franka_reg_G_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,43>>(buffer);
}

// Regressor matrix of the quantity J^T*w
Eigen::Matrix<double,7,6> thunder_franka::get_reg_JTw() {
	thread_local double buffer[42];
	thread_local long long p3[franka_reg_JTw_fun_SZ_IW];
	thread_local double p4[franka_reg_JTw_fun_SZ_W];
	const double* input_[] = {q.data(), w.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_reg_JTw_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,6>>(buffer);
}

// Regressor matrix of the quantity J*dq
Eigen::Matrix<double,6,6> thunder_franka::get_reg_Jdq() {
	thread_local double buffer[36];
	thread_local long long p3[franka_reg_Jdq_fun_SZ_IW];
	thread_local double p4[franka_reg_Jdq_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = franka_reg_Jdq_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,6>>(buffer);
}

// Regressor matrix of term M*ddqr
Eigen::Matrix<double,7,70> thunder_franka::get_reg_M() {
	thread_local double buffer[490];
	thread_local long long p3[franka_reg_M_fun_SZ_IW];
	thread_local double p4[franka_reg_M_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data()};
	double* output_[] = {buffer};
	int check = franka_reg_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,70>>(buffer);
}

// Regressor of masses defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,7,43> thunder_franka::get_reg_M_red() {
	thread_local double buffer[301];
	thread_local long long p3[franka_reg_M_red_fun_SZ_IW];
	thread_local double p4[franka_reg_M_red_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data()};
	double* output_[] = {buffer};
	int check = franka_reg_M_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,43>>(buffer);
}

// Regressor matrix of the link friction
Eigen::Matrix<double,7,14> thunder_franka::get_reg_dl() {
	thread_local double buffer[98];
	thread_local long long p3[franka_reg_dl_fun_SZ_IW];
	thread_local double p4[franka_reg_dl_fun_SZ_W];
	const double* input_[] = {dq.data()};
	double* output_[] = {buffer};
	int check = franka_reg_dl_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,14>>(buffer);
}


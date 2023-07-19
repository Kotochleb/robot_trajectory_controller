#include "pendulum.hpp"

#include <cassert>

/* Constructor. */
InvertedPendulum::InvertedPendulum() {}

InvertedPendulum::~InvertedPendulum() {}

bool InvertedPendulum::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m,
                                    Ipopt::Index& nnz_jac_g,
                                    Ipopt::Index& nnz_h_lag,
                                    Ipopt::TNLP::IndexStyleEnum& index_style) {

  index_style = Ipopt::TNLP::IndexStyleEnum::FORTRAN_STYLE;

  return true;
}

bool InvertedPendulum::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l,
                                       Ipopt::Number* x_u, Ipopt::Index m,
                                       Ipopt::Number* g_l, Ipopt::Number* g_u) {
  return true;
}

bool InvertedPendulum::get_starting_point(Ipopt::Index n, bool init_x,
                                          Ipopt::Number* x, bool init_z,
                                          Ipopt::Number* z_L,
                                          Ipopt::Number* z_U, Ipopt::Index m,
                                          bool init_lambda,
                                          Ipopt::Number* lambda) {
  return true;
}

bool InvertedPendulum::eval_f(Ipopt::Index n, const Ipopt::Number* x,
                              bool new_x, Ipopt::Number& obj_value) {

  return true;
}

bool InvertedPendulum::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x,
                                   bool new_x, Ipopt::Number* grad_f) {
  return true;
}

bool InvertedPendulum::eval_g(Ipopt::Index n, const Ipopt::Number* x,
                              bool new_x, Ipopt::Index m, Ipopt::Number* g) {
  return true;
}

bool InvertedPendulum::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x,
                                  bool new_x, Ipopt::Index m,
                                  Ipopt::Index nele_jac, Ipopt::Index* iRow,
                                  Ipopt::Index* jCol, Ipopt::Number* values) {
  return true;
}

bool InvertedPendulum::eval_h(Ipopt::Index n, const Ipopt::Number* x,
                              bool new_x, Ipopt::Number obj_factor,
                              Ipopt::Index m, const Ipopt::Number* lambda,
                              bool new_lambda, Ipopt::Index nele_hess,
                              Ipopt::Index* iRow, Ipopt::Index* jCol,
                              Ipopt::Number* values) {
  return true;
}

void InvertedPendulum::finalize_solution(
    Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
    const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
    const Ipopt::Number* g, const Ipopt::Number* lambda,
    Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {}

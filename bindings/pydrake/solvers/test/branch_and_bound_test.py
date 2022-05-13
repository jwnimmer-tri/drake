import unittest
import time

import numpy as np

from pydrake.symbolic import Polynomial, Variables
from pydrake.solvers.mathematicalprogram import MathematicalProgram


class TestFoo(unittest.TestCase):
    def test(self):
        nz = 3
        deg = 16
        prog = MathematicalProgram()
        z = prog.NewIndeterminates(nz, "z")
        J = prog.NewFreePolynomial(Variables(z), deg)
        J_expr = J.ToExpression()
        dJdz = J_expr.Jacobian(z)
        f2 = np.array([[0], [0], [1]])
        u_opt = -0.5 * (f2.T).dot(dJdz.T)
        z0 = np.array([0, 1, 0])
        ell = (z-z0).dot(z-z0) + u_opt.dot(u_opt)
        f = np.array([
                    z[1] * z[2],
                    - z[0] * z[2],
                    (z[0] + u_opt[0] - z[2])])
        RHS = ell + dJdz.dot(f)
        start_time = time.time()
        Polynomial(RHS, Variables(z))
        end_time = time.time()
        print("Polynomial parse time: ", end_time-start_time)

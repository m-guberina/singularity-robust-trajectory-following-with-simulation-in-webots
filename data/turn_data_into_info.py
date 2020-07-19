import numpy as np
import matplotlib.pyplot as plt


def calcAvgManipAndMinEigVal(data_file):
    data_f = open(data_file, "r")
    data_lines = data_f.readlines()
    total_manip = 0.0
    eigvals = []
    n_of_used_lines = 0

    for i in range(len(data_lines)):
        if i < 100:
            continue
        n_of_used_lines += 1
        line = data_lines[i].rstrip().split(";")
        manip = line[0]
        eigval = line[1]
        total_manip += float(manip)
        eigvals.append(float(eigval))

    avg_manip = total_manip / n_of_used_lines
    min_eigval = sorted(eigvals)[0]
    data_f.close()
    return [avg_manip, min_eigval]



fig = plt.figure()
#ax = fig.add_axes([0,0,1,1])
methods = ["transpose", "pseudoinverse", "damped squares", "sing.-robust QP"]

transp = calcAvgManipAndMinEigVal("./transpose_data_1")
pinv = calcAvgManipAndMinEigVal("./pinv_data_1")
damped_squares = calcAvgManipAndMinEigVal("./damped_squares_data_1")
sing_robust_qp = calcAvgManipAndMinEigVal("./sing_avoidance_data_1")

avg_manips = [transp[0], pinv[0], damped_squares[0], sing_robust_qp[0]]
min_eigvals = [transp[1], pinv[1], damped_squares[1], sing_robust_qp[1]]

plt.bar(methods, min_eigvals)
#plt.ylabel('Average manipulability')
plt.ylabel('Eigenvalues')
plt.title('Smallest eigenvalue')
plt.show()

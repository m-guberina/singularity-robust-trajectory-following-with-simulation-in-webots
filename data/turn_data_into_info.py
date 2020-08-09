import numpy as np
import matplotlib.pyplot as plt
import os


def calcAvgManipAndMinEigVal(data_file):
    data_f = open(data_file, "r")
    data_lines = data_f.readlines()
    total_manip = 0.0
    all_min_eigenvals = 0.0
    all_max_eigenvals = 0.0
    n_of_used_lines = 0

    for i in range(len(data_lines)):
        n_of_used_lines += 1
        line = data_lines[i].rstrip().split(";")
        manip = line[0]
        min_eigval = line[1]
        max_eigval = line[2]
        total_manip += float(manip)
        all_min_eigenvals += float(min_eigval)
        all_max_eigenvals += float(max_eigval)

    avg_manip = total_manip / n_of_used_lines
    avg_min_eigenval = all_min_eigenvals / n_of_used_lines
    avg_max_eigenval = all_max_eigenvals / n_of_used_lines
    data_f.close()
    return [avg_manip, avg_min_eigenval, avg_max_eigenval]



fig = plt.figure(1)
#ax = fig.add_axes([0,0,1,1])
methods = ["no sing. avoidance", "E = kM", "E = kI", "manip. max."]


current_dir = os.popen("pwd").read().strip().split("/")
if current_dir[-1] == "data":
    prefix = "./"
else:
    prefix = "./data/"

no_avoid = calcAvgManipAndMinEigVal(prefix + "no_sing_avoid_200_inv_kinms")
E_kI = calcAvgManipAndMinEigVal(prefix + "E_kI_200_inv_kinms")
E_kM = calcAvgManipAndMinEigVal(prefix + "E_kM_200_inv_kinms")
manip_max = calcAvgManipAndMinEigVal(prefix + "ManipMax_200_inv_kinms")

avg_manips = [no_avoid[0], E_kM[0], E_kI[0], manip_max[0]]
avg_min_eigvals = [no_avoid[1], E_kM[1], E_kI[1], manip_max[1]]
avg_max_eigvals = [no_avoid[2], E_kM[2], E_kI[2], manip_max[2]]


# first graph: average manipulability  measure

#plt.bar(methods, avg_max_eigvals)
plt.bar(methods, avg_manips)
#plt.ylabel('Average manipulability')
plt.ylabel('Manipulability measure')
plt.title('Average manipubility index')

# second graph: average minimum eigenvalue
fig = plt.figure(2)
#plt.ylabel('Average manipulability')
plt.bar(methods, avg_min_eigvals)
plt.ylabel('Eigenvalues')
plt.title('Average smallest eigenvalue')



# second graph: average minimum eigenvalue
fig = plt.figure(3)
#plt.ylabel('Average manipulability')
plt.bar(methods, avg_max_eigvals)
plt.ylabel('Eigenvalues')
plt.title('Average highest eigenvalue')


plt.show()

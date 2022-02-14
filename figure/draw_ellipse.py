from tokenize import Double
import numpy as np
import matplotlib
# switch to pgf backend
matplotlib.use('pgf')
# import matplotlib
import matplotlib.pyplot as plt

# update latex preamble
plt.rcParams.update({
    "font.family": "serif",
    "text.usetex": True,
    "pgf.rcfonts": False,
    "pgf.texsystem": 'pdflatex', # default is xetex
    "pgf.preamble": [
         r"\usepackage[T1]{fontenc}",
         r"\usepackage{mathpazo}"
         ]
})

from scipy.stats import chi2
from matplotlib import patches

p = 2 # degrees of freedom
alpha = 0.05 # 95% confidence rate
s = chi2.isf(alpha, p) # the scale of the ellipse
print(s)

# center of ellipse
x_mean = np.array([0.0, 0.0])
# covariance matrix
cov = [[0.0004, -6.81116942e-15],
     [-6.81116942e-15, 0.0004]]
eigenvalues, eigenvectors = np.linalg.eig(cov)
j_max = np.argmax(eigenvalues)
j_min = np.argmin(eigenvalues)
# plot
fig= plt.figure()
ax = plt.gca()
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.plot(x_mean[0], x_mean[1], 'C3o', label='mean value', marker=".", markersize=4)
first_legend = plt.legend(loc='upper left')

ell = patches.Ellipse((x_mean[0], x_mean[1]), 
                      2.0*np.sqrt(s*eigenvalues[j_max]), 2.0*np.sqrt(s*eigenvalues[j_min]),
                      angle=np.arctan2(eigenvectors[j_max,1], eigenvectors[j_max,0])*180/np.pi, 
                      alpha=0.2, color='C0')
ax.add_artist(ell)
plt.axis('equal')
ax.add_artist(first_legend)
plt.legend([ell],[str(100*(1-alpha)) + ' \% confidence'], loc='lower right')
plt.xlim([-0.5, 0.5])
plt.ylim([-0.5, 0.5])

plt.savefig('example.pdf', format='pdf')


# read the 2x2 covariance matrices result from gloc_node.cpp 
def conv(fld):
    return -float(fld[:-1]) if fld.endswith(b'-') else float(fld)
xy_matrix = np.loadtxt("cov_xy.txt", converters={0: conv, 1: conv})
# print(xy_matrix)
lm_matrix = np.loadtxt("cov_lm.txt", converters={0: conv, 1: conv})
# print(lm_matrix)
pose_xy = np.loadtxt("x_result.txt", delimiter=",", converters={0: conv, 1: conv})
# print(pose_xy)
pose_lm = np.loadtxt("lm_result.txt", delimiter=",", converters={0: conv, 1: conv})
# print(pose_lm)
box =np.loadtxt("box.txt", delimiter=",", converters={0: conv, 1: conv})
# print(box)

# f= open('lm_result.txt', 'r')
# data_txt = f.read()
# data_txt = data_txt.replace('(', '')
# data_txt = data_txt.replace(')', '')
# print(data_txt)


# plot
fig1= plt.figure()
ax1 = plt.gca()
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')

# plot intervl boxes of robot poses
# for i in range(400):
#     left, bottom, width, height =(box[i, :][0], box[i, :][1], box[i, :][2], box[i, :][3])
#     rect = patches.Rectangle((left, bottom), width, height, fill=False, color='green', linewidth=0.4 )
#     ax1.add_patch(rect)

# plot ellipses of robot poses
for i in range(400):
    cov_pose = xy_matrix[[2*i  , 2*i+1], :]
    # print(cov_pose)
    mean = pose_xy[i, :]
    # print(mean[1])

    eigenvalues1, eigenvectors1 = np.linalg.eig(cov_pose)
    j_max1 = np.argmax(eigenvalues1)
    j_min1 = np.argmin(eigenvalues1)
    reddot, =plt.plot(mean[0], mean[1], 'ro', label='mean value', marker=".", markersize=1)
    ell1 = patches.Ellipse((mean[0], mean[1]), 
                      2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
                      angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
                      edgecolor='C1', lw=0.4, facecolor='none')
    
    # ell1 = patches.Ellipse((mean[0], mean[1]), 
    #                   2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
    #                   angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
    #                   edgecolor='r', lw=0.3, alpha=0.5, facecolor='r')
    ax1.add_artist(ell1)

# plot ellipses of landmarks
for j in range(169):
    cov_lm = lm_matrix[[2*j  , 2*j+1], :]
    # print(cov_pose)
    mean_lm = pose_lm[j, :]
    # print(mean[1])

    eigenvalues2, eigenvectors2 = np.linalg.eig(cov_lm)
    j_max2 = np.argmax(eigenvalues2)
    j_min2 = np.argmin(eigenvalues2)
    bluedot, = plt.plot(mean_lm[0], mean_lm[1], 'bo', label='mean value', marker=".", markersize=1)
    ell2 = patches.Ellipse((mean_lm[0], mean_lm[1]), 
                      2.0*np.sqrt(s*eigenvalues2[j_max2]), 2.0*np.sqrt(s*eigenvalues2[j_min2]),
                      angle=np.arctan2(eigenvectors2[j_max2,1], eigenvectors2[j_max2,0])*180/np.pi, 
                      edgecolor='C9', lw=0.4, alpha=0.5, facecolor='C9')
    ax1.add_artist(ell2)


plt.axis('equal')
# plt.legend([rect, ell1, ell2],['interval box', str(100*(1-alpha)) + ' \% confidence ellipse ', 
#                          str(100*(1-alpha)) + ' \% confidence ellipse '], loc='upper left')
plt.legend([reddot, ell1, bluedot, ell2],['probabilistic solution of robot pose', str(100*(1-alpha)) + ' \% confidence ellipse of robot pose', 
                         'landmark position', str(100*(1-alpha)) + ' \% confidence ellipse of landmark position'], loc='upper left')
plt.xlim([-2, 12])
plt.ylim([-12, 6])
# plt.xticks(np.arange(-4, 16, 1))
# plt.yticks(np.arange(-14, 6, 1))
# ax1.grid(which='major', color='#DDDDDD', linewidth=0.2)
# ax1.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.2)
# ax1.minorticks_on()
plt.savefig('example1.pdf', format='pdf')
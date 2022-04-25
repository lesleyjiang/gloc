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
# import plottools as pt
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset, inset_axes


p = 2 # degrees of freedom
alpha = 0.05 # 95% confidence rate
s = chi2.isf(alpha, p) # the scale of the ellipse
print(s)




# read the 2x2 covariance matrices result from gloc_node.cpp 
def conv(fld):
    return -float(fld[:-1]) if fld.endswith(b'-') else float(fld)
xy_matrix = np.loadtxt("cov_x15.txt", converters={0: conv, 1: conv})
# print(xy_matrix)
lm_matrix = np.loadtxt("cov_lm15.txt", converters={0: conv, 1: conv})
# print(lm_matrix)

pose_xy = np.loadtxt("x15.txt", delimiter=",", converters={0: conv, 1: conv})
# print(pose_xy)
pose_lm = np.loadtxt("lm15.txt", delimiter=",", converters={0: conv, 1: conv})
# print(pose_lm)

box =np.loadtxt("box15.txt", delimiter=",", converters={0: conv, 1: conv})
# print(box)


pose_xy_s1 = np.loadtxt("x17.txt", delimiter=",", converters={0: conv, 1: conv})
pose_lm_s1 = np.loadtxt("lm17.txt", delimiter=",", converters={0: conv, 1: conv})
xy_matrix_s1 = np.loadtxt("cov_x17.txt", converters={0: conv, 1: conv})
lm_matrix_s1 = np.loadtxt("cov_lm17.txt", converters={0: conv, 1: conv})



# plot
fig1= plt.figure()
ax1 = plt.gca()
plt.xlabel(r'$x\ in\ m$')
plt.ylabel(r'$y\ in\ m$')

# plot the corridor
corridor_in = patches.Rectangle((1, -9), 8, 8, fill=False, color='black', linewidth=0.8 )
corridor_out = patches.Rectangle((-1, -11), 12, 12, fill=False, color='black', linewidth=0.8 )
ax1.add_patch(corridor_in)
ax1.add_patch(corridor_out)


# plot intervl boxes of robot poses
for i in range(400):
    left, bottom, width, height =(box[i, :][0], box[i, :][1], box[i, :][2], box[i, :][3])
    rect = patches.Rectangle((left, bottom), width, height, fill=False, color='green', linewidth=0.4 )
    ax1.add_patch(rect)

# plot ellipses of robot poses
for i in range(400):
    cov_pose = xy_matrix[[2*i  , 2*i+1], :]
    # print(cov_pose)
    mean = pose_xy[i, :]
    # print(mean[1])

    eigenvalues1, eigenvectors1 = np.linalg.eig(cov_pose)
    j_max1 = np.argmax(eigenvalues1)
    j_min1 = np.argmin(eigenvalues1)
    # bdot, =plt.plot(mean[0], mean[1], 'ko', label='mean value', marker=".", markersize=1)
    # ell1 = patches.Ellipse((mean[0], mean[1]), 
    #                   2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
    #                   angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
    #                   edgecolor='C1', lw=0.4, facecolor='none')
    
    ell1 = patches.Ellipse((mean[0], mean[1]), 
                      2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
                      angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
                      edgecolor='r', lw=0.2, alpha=0.8, facecolor='r')
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
    # bluedot, = plt.plot(mean_lm[0], mean_lm[1], 'bo', label='mean value', marker=".", markersize=1)
    ell2 = patches.Ellipse((mean_lm[0], mean_lm[1]), 
                      2.0*np.sqrt(s*eigenvalues2[j_max2]), 2.0*np.sqrt(s*eigenvalues2[j_min2]),
                      angle=np.arctan2(eigenvectors2[j_max2,1], eigenvectors2[j_max2,0])*180/np.pi, 
                      edgecolor='C4', lw=0.2, alpha=0.9, facecolor='C9')
    ax1.add_artist(ell2)



# plot ellipses of robot poses with systematic error (1 sigma)
for i in range(400):
    cov_pose_s1 = xy_matrix_s1[[2*i  , 2*i+1], :]
    mean_s1 = pose_xy_s1[i, :]
    # mean = pose_xy[i, :]

    eigenvalues_s1, eigenvectors_s1 = np.linalg.eig(cov_pose_s1)
    j_max_s1 = np.argmax(eigenvalues_s1)
    j_min_s1 = np.argmin(eigenvalues_s1)
    # bdot_s1, =plt.plot(mean[0], mean[1], 'k', label='mean value', marker=".", markersize=1)
    # ell_s1 = patches.Ellipse((mean_s1[0], mean_s1[1]), 
    #                   2.0*np.sqrt(s*eigenvalues_s1[j_max_s1]), 2.0*np.sqrt(s*eigenvalues_s1[j_min_s1]),
    #                   angle=np.arctan2(eigenvectors_s1[j_max_s1,1], eigenvectors_s1[j_max_s1,0])*180/np.pi, 
    #                   edgecolor='C1', lw=0.4, facecolor='none')
    
    # mdot_s1, =plt.plot(mean_s1[0], mean_s1[1], 'magenta', label='mean value', marker=".", markersize=1)
    ell_s1 = patches.Ellipse((mean_s1[0], mean_s1[1]), 
                      2.0*np.sqrt(s*eigenvalues_s1[j_max_s1]), 2.0*np.sqrt(s*eigenvalues_s1[j_min_s1]),
                      angle=np.arctan2(eigenvectors_s1[j_max_s1,1], eigenvectors_s1[j_max_s1,0])*180/np.pi, 
                      edgecolor='orange', lw=0.2, alpha=0.8, facecolor='orange')
    ax1.add_artist(ell_s1)

# plot ellipses of landmarks with systematic error (1 sigma)
for j in range(169):
    cov_lm_s1 = lm_matrix_s1[[2*j  , 2*j+1], :]
    # print(cov_pose)
    mean_lm_s1 = pose_lm_s1[j, :]
    # print(mean[1])

    eigenvalues_lm_s1, eigenvectors_lm_s1 = np.linalg.eig(cov_lm_s1)
    j_max_lm_s1 = np.argmax(eigenvalues_lm_s1)
    j_min_lm_s1 = np.argmin(eigenvalues_lm_s1)
    # bluedot_s1, = plt.plot(mean_lm_s1[0], mean_lm_s1[1], 'bo', label='mean value', marker=".", markersize=1)
    # ell_lm_s1 = patches.Ellipse((mean_lm_s1[0], mean_lm_s1[1]), 
    #                   2.0*np.sqrt(s*eigenvalues_lm_s1[j_max_lm_s1]), 2.0*np.sqrt(s*eigenvalues_lm_s1[j_min_lm_s1]),
    #                   angle=np.arctan2(eigenvectors_lm_s1[j_max_lm_s1,1], eigenvectors_lm_s1[j_max_lm_s1,0])*180/np.pi, 
    #                   edgecolor='C7', lw=0.2, alpha=1, facecolor='C7')

    ell_lm_s1 = patches.Ellipse((mean_lm_s1[0], mean_lm_s1[1]), 
                      2.0*np.sqrt(s*eigenvalues_lm_s1[j_max_lm_s1]), 2.0*np.sqrt(s*eigenvalues_lm_s1[j_min_lm_s1]),
                      angle=np.arctan2(eigenvectors_lm_s1[j_max_lm_s1,1], eigenvectors_lm_s1[j_max_lm_s1,0])*180/np.pi, 
                      edgecolor='C9', lw=0.2, alpha=0.9, facecolor='C9')

    ax1.add_artist(ell_lm_s1)


plt.axis('equal')
# plt.legend([ell1, ell2],[str(100*(1-alpha)) + ' \% confidence ellipse ', 
#                          str(100*(1-alpha)) + ' \% confidence ellipse '], loc='upper left')

plt.legend([rect, ell1],['Interval', ' Factor graph'], loc='upper left')
# plt.legend([rect, ell_s1],['Interval', ' Factor graph'], loc='upper left')
# plt.legend([rect, ell1, ell_s1],['Interval-1$\sigma$', ' Factor graph$', ' Factor graph-1$\sigma$'], loc='upper left')

# plt.legend([rect, ell1, ell2],['interval box', str(100*(1-alpha)) + ' \% confidence ellipse ', 
#                          str(100*(1-alpha)) + ' \% confidence ellipse '], loc='upper left')
# plt.legend([rect, ell1, ell_s1],['interval box', str(100*(1-alpha)) + ' \% confidence ellipse ', 
#                          str(100*(1-alpha)) + ' \% confidence ellipse '], loc='upper left')

# plt.legend([reddot, ell1, bluedot, ell2],['probabilistic solution of robot pose', str(100*(1-alpha)) + ' \% confidence ellipse of robot pose', 
#                          'landmark position', str(100*(1-alpha)) + ' \% confidence ellipse of landmark position'], loc='upper left')

plt.xlim([-2, 11])
plt.ylim([-12, 3.5])

# plt.axis('off')





# # Zoom in plot

# plt2=fig1.add_subplot(2, 2, (1, 2))

# # axins1 = zoomed_inset_axes(ax1, zoom = 4, loc=4)

# x1, x2, y1, y2 = 3,7,-10.5,-9.5
# for i in range(400):
#     left, bottom, width, height =(box[i, :][0], box[i, :][1], box[i, :][2], box[i, :][3])
#     rect1 = patches.Rectangle((left, bottom), width, height, fill=False, color='green', linewidth=0.4 )
#     plt2.add_patch(rect1)

# # # plot ellipses of robot poses
# # for i in range(400):
# #     cov_pose = xy_matrix[[2*i  , 2*i+1], :]
# #     # print(cov_pose)
# #     mean = pose_xy[i, :]
# #     # print(mean[1])

# #     eigenvalues1, eigenvectors1 = np.linalg.eig(cov_pose)
# #     j_max1 = np.argmax(eigenvalues1)
# #     j_min1 = np.argmin(eigenvalues1)
# #     blackdot, =plt.plot(mean[0], mean[1], '-ok', label='mean value', marker=".", markersize=2)
# #     # ell1 = patches.Ellipse((mean[0], mean[1]), 
# #     #                   2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
# #     #                   angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
# #     #                   edgecolor='C1', lw=0.4, facecolor='none')
    
# #     ell11 = patches.Ellipse((mean[0], mean[1]), 
# #                       2.0*np.sqrt(s*eigenvalues1[j_max1]), 2.0*np.sqrt(s*eigenvalues1[j_min1]),
# #                       angle=np.arctan2(eigenvectors1[j_max1,1], eigenvectors1[j_max1,0])*180/np.pi, 
# #                       edgecolor='r', lw=0.2, alpha=1, facecolor='r')
# #     plt2.add_artist(ell11)

# # plot ellipses of robot poses with systematic error (1 sigma)
# for i in range(400):
#     cov_pose_s1 = xy_matrix_s1[[2*i  , 2*i+1], :]
#     mean_s1 = pose_xy_s1[i, :]
#     mean = pose_xy[i, :]

#     eigenvalues_s1, eigenvectors_s1 = np.linalg.eig(cov_pose_s1)
#     j_max_s1 = np.argmax(eigenvalues_s1)
#     j_min_s1 = np.argmin(eigenvalues_s1)
#     # reddot_s1, =plt.plot(mean[0], mean[1], 'k', label='mean value', marker=".", markersize=4)
#     # hdot_s1, =plt.plot(mean_s1[0], mean_s1[1], 'cyan', label='mean value', marker=".", markersize=3)
#     # ell_s1 = patches.Ellipse((mean_s1[0], mean_s1[1]), 
#     #                   2.0*np.sqrt(s*eigenvalues_s1[j_max_s1]), 2.0*np.sqrt(s*eigenvalues_s1[j_min_s1]),
#     #                   angle=np.arctan2(eigenvectors_s1[j_max_s1,1], eigenvectors_s1[j_max_s1,0])*180/np.pi, 
#     #                   edgecolor='C1', lw=0.4, facecolor='none')
    
#     ell_s11 = patches.Ellipse((mean_s1[0], mean_s1[1]), 
#                       2.0*np.sqrt(s*eigenvalues_s1[j_max_s1]), 2.0*np.sqrt(s*eigenvalues_s1[j_min_s1]),
#                       angle=np.arctan2(eigenvectors_s1[j_max_s1,1], eigenvectors_s1[j_max_s1,0])*180/np.pi, 
#                       edgecolor='orange', lw=0.2, alpha=1, facecolor='orange')
#     plt2.add_artist(ell_s11)


# plt2.axis('equal')
# plt2.set_xlim(x1, x2)
# plt2.set_ylim(y1, y2)
# plt2.set_xlabel(r'$x\ in\ m$')
# plt2.set_ylabel(r'$y\ in\ m$')
# # plt2.xticks()
# # plt2.yticks()
# # mark_inset(ax1, axins1, loc1=2, loc2=1, fc="none", ec="0.5")



plt.savefig('example26.pdf', format='pdf')
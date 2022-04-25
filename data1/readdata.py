import numpy as np

f= open('x_result.txt', 'r')
data_txt = f.read()
data_txt = data_txt.replace('(', '')
data_txt = data_txt.replace(')', '')
print(data_txt)
f= open('x17.txt', 'w')
f.write(data_txt)

f= open('lm_result.txt', 'r')
data_txt = f.read()
data_txt = data_txt.replace('(', '')
data_txt = data_txt.replace(')', '')
print(data_txt)
f= open('lm17.txt', 'w')
f.write(data_txt)

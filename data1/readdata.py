import numpy as np

f= open('x_result.txt', 'r')
data_txt = f.read()
data_txt = data_txt.replace('(', '')
data_txt = data_txt.replace(')', '')
print(data_txt)
f= open('x6.txt', 'w')
f.write(data_txt)

f= open('lm_result.txt', 'r')
data_txt = f.read()
data_txt = data_txt.replace('(', '')
data_txt = data_txt.replace(')', '')
print(data_txt)
f= open('lm6.txt', 'w')
f.write(data_txt)

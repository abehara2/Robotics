import modern_robotics as mr
import numpy as np
import sympy as sym

print("Question 1 \n")
s1 = np.array([0,0,0,1,0,0])
s2 = np.array([0,0,0,0,-1,0])
s3 = np.array([0,0,0,0,-1,0])

s1 = np.transpose(s1)
s2 = np.transpose(s2)
s3 = np.transpose(s3)
Slist = np.array([s1,s2,s3])

JS = mr.JacobianSpace(Slist, thetalist)
print(JS)

print("\nQuestion 2 \n")
theta = np.array([[0.95], [0.98], [-0.92]])

b1 = np.array([1,0,0,0,0,0]) # -4,0,0
b2 = np.array([1,0,0,0,0,0])  #-6,0,0
b3 = np.array([0,0,0,1,0,0]) # 0 -2 t

Blist = np.array([b1,b2,b3])
Blist = np.transpose(Blist)

JB = mr.JacobianBody(Blist, theta)
print(JB)

print("\n Question 3 \n")

T_1in0 = np.array([[0.85174056, 0.41765295, 0.31639222, 3.70016378], [0.50645468, -0.81103411, -0.29279229, -7.42921783], [0.13431932, 0.40962139, -0.90231294, 4.30158108], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
M = np.array([[0.00000000, 1.00000000, 0.00000000, 6.00000000], [1.00000000, 0.00000000, 0.00000000, -4.00000000], [0.00000000, 0.00000000, -1.00000000, 0.00000000], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
S = np.array([[0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000], [-1.00000000, -1.00000000, 0.00000000, 0.00000000, 0.00000000,  1.00000000], [0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000], [0.00000000, 0.00000000, 0.00000000, -6.00000000, 0.00000000, 0.00000000], [0.00000000, 0.00000000, 0.00000000, -2.00000000, 1.00000000, 0.00000000], [-2.00000000, 0.00000000, 6.00000000, 0.00000000, 0.00000000, 4.00000000]])
print(mr.IKinSpace(S, M,T_1in0, [0,0,0,0,0,0],0.01,0.01))

print("\n question 4 \n")

s1 = np.array([0,0,1,0,2,0]) #-2,0,0
s2 = np.array([0,1,0,0,0,-2]) #-2,2,0
s3 = np.array([0,0,1,4,2,0]) # -2,4,0
s4 = np.array([0,1,0,0,0,-2]) #-2,6,0
s5 = np.array([0,-1,0,0,0,0]) # 0,6,0
s6 = np.array([0,0,0,1,0,0])

#rotate x by +90
#rotate y by 90

T_1in0 = np.array([[-0.53170421, 0.40925994, -0.74148293, -2.33801009], [-0.42110581, 0.63185226, 0.65071701, 5.63880461], [0.73482007, 0.65823174, -0.16361677, 2.39935579], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])

Slist = np.array([s1,s2,s3, s4, s5, s6])
Slist = np.transpose(Slist)
print(Slist)

# y -90
# x -90
M = np.eye(4)
M[0] = [0,1,0, 2]
M[1] = [0,0,1, 4]
M[2] = [1,0,0, 0]
print(M)

print( mr.IKinSpace(Slist, M, T_1in0, [0,0,0,0,0,0], 0.01, 0.01))
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import random
import pulp
import gurobipy as gp
from gurobipy import GRB


# Gurobi class for vehicle routing

class CVRP_SPD_GUROBI:
	def __init__(self, numberOfCustomers, numberOfVehicles, capacityOfVehicle, pickups, delivery, costMatrix):
		self.numberOfCustomers = numberOfCustomers
		self.numberOfVehicles  = numberOfVehicles
		self.capacityOfVehicle = capacityOfVehicle
		self.pickups = pickups
		self.delivery = delivery
		self.costMatrix        = costMatrix
		self.initializeLP()
	
	def initializeLP(self):
		self.cvrpLP = gp.Model("CVRP_PD")
		objective = None
		x, y, z = [], [], []
		D, d = [], []
		P, p = [], []
		
		K = self.numberOfVehicles
		N = len(costMatrix)

		# create variables
		for i in range(N):
			xTemp1 = []
			for j in range(N):
				xTemp2 = []
				for k in range(K):
					xTemp3 = self.cvrpLP.addVar(name='x('+str(i)+','+str(j)+','+str(k)+')', vtype=GRB.BINARY, lb=0, ub=1)
					xTemp2.append(xTemp3)
				xTemp1.append(xTemp2)
			x.append(xTemp1)
		print(type(x[0][0][0]))
		
		for i in range(N):
			yTemp1 = []
			for j in range(N):
				yTemp2 = self.cvrpLP.addVar(name='y('+str(i)+','+str(j)+')', vtype=GRB.CONTINUOUS, lb=0)
				yTemp1.append(yTemp2)
			y.append(yTemp1)
		
		for i in range(N):
			zTemp1 = []
			for j in range(N):
				zTemp2 = self.cvrpLP.addVar(name='z('+str(i)+','+str(j)+')', vtype=GRB.CONTINUOUS, lb=0)
				zTemp1.append(zTemp2)
			z.append(zTemp1)
		
		# objective function
		for i in range(N):
			for j in range(N):
				for k in range(K):
					objective += self.costMatrix[i][j] * x[i][j][k]
		self.cvrpLP.setObjective(objective, GRB.MINIMIZE)

		#constraints

		#constraint-1
		for i in range(1, N):
			constraint1 = None
			for j in range(N):
				for k in range(K):
					if(constraint1 == None):
						constraint1 = x[i][j][k]
					else:
						constraint1 = constraint1 + x[i][j][k]
			self.cvrpLP.addConstr(constraint1 == 1)
		
		#constraint-2
		for k in range(K):
			for i in range(N):
				constraint2a, constraint2b = None, None
				for j in range(N):
					if(constraint2a == None):
						constraint2a = x[i][j][k]
					else:
						constraint2a = constraint2a + x[i][j][k]
					if(constraint2b == None):
						constraint2b = x[i][j][k]
					else:
						constraint2b = constraint2b + x[i][j][k]
					# constraint2a += x[i][j][k]
					# constraint2b += x[j][i][k]
				self.cvrpLP.addConstr(constraint2a - constraint2b == 0)
		
		#constraint-3
		for k in range(K):
			constraint3 = None
			for i in range(N):
				if(constraint3 == None):
						constraint3 = x[0][i][k]
				else:
					constraint3 = constraint3 + x[0][i][k]
				# constraint3 += x[0][i][k]
			self.cvrpLP.addConstr(constraint3 <= 1)
		
		# constraint-4
		for i in range(N):
			for j in range(N):
				constraint4a = None
				for k in range(K):
					if(constraint4a == None):
						constraint4a = x[i][j][k]
					else:
						constraint4a = constraint4a + x[i][j][k]
				self.cvrpLP.addConstr(y[i][j] + z[i][j] <= self.capacityOfVehicle * constraint4a)
		
		#constraint-5
		for i in range(1, N):
			constraint5a, constraint5b = None, None
			for j in range(N):
				if(constraint5a == None):
					constraint5a = y[i][j]
				else:
					constraint5a = constraint5a + y[i][j]
				if(constraint5b == None):
					constraint5b = y[j][i]
				else:
					constraint5b = constraint5b + y[j][i]
			self.cvrpLP.addConstr(constraint5a - constraint5b == self.pickups[i])

		#constraint-6
		for i in range(1, N):
			constraint6a, constraint6b = None, None
			for j in range(N):
				if(constraint6a == None):
					constraint6a = z[j][i]
				else:
					constraint6a = constraint6a + z[j][i]
				if(constraint6b == None):
					constraint6b = z[i][j]
				else:
					constraint6b = constraint6b + z[i][j]
			self.cvrpLP.addConstr(constraint6a - constraint6b == self.delivery[i])
		
		print(self.cvrpLP)

	def solve(self):
		status = self.cvrpLP.optimize()#pulp.solvers.PULP_CBC_CMD(self.cvrpLP))
		print(status)
	
	def getResult(self):
		print("Objective value: ", self.cvrpLP.ObjVal)
		for v in self.cvrpLP.getVars():
			print(v.varName, " = ", v.x)

costMatrix = [[0,9,14,23,32,50,21,49,30,27,35,28,18],
[9,0,21,22,36,52,24,51,36,37,41,30,20],
[14,21,0,25,38,5,31,7,36,43,29,7,6],
[23,22,25,0,42,12,35,17,44,31,31,11,6],
[32,36,38,42,0,22,37,16,46,37,29,13,14],
[50,52,5,12,22,0,41,23,10,39,9,17,16],
[21,24,31,35,37,41,0,26,21,19,10,25,12],
[49,51,7,17,16,23,26,0,30,28,16,27,12],
[30,36,36,44,46,10,21,30,0,25,22,10,20],
[27,37,43,31,37,39,19,28,25,0,20,16,8],
[35,41,29,31,29,9,10,16,22,20,0,10,10],
[28,30,7,11,13,17,25,27,10,16,10,0,10],
[18,20, 6, 6,14,16,12,12,20,8, 10,10,0]]

delivery = [0, 1200, 1700, 1500, 1400, 1700, 1400, 1200, 1900, 1800, 1600, 1700, 1100]
pickups = [1100, 0, 1200, 1700, 1500, 1400, 1700, 1400, 1200, 1900, 1800, 1600, 1700]
capacityOfVehicle = 6000
numberOfVehicles = 4
numberOfCustomers = len(costMatrix) - 1


lp = CVRP_SPD_GUROBI(numberOfCustomers, numberOfVehicles, capacityOfVehicle, pickups, delivery, costMatrix)
lp.solve()
lp.getResult()
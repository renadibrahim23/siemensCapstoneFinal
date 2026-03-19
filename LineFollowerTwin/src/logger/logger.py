#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math
import csv
import matplotlib.pyplot as plt

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.pos_x = 0
		self.pos_y = 0
		self.theta = 0
		self.angular_w = 0
		self.lateral_error = 0
		self.theta_error = 0


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

# End of user custom code region. Please don't edit beyond this point.
class Logger:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor
		self.args = args
		self.filename = f"results_{args.label}.csv"
		self.csv_file = open(self.filename, mode='w', newline='')
		self.writer = csv.writer(self.csv_file)

		self.writer.writerow([
			"time_s", "pos_x", "pos_y", "theta", "lateral_error", "angular_w", "target_y"
		])

		# Real-time Plotting Init
		self.time_history = []
		self.pos_y_history = []
		self.target_y_history = [] 
		self.plot_decimation = 20 
		self.counter = 0

		plt.ion() 
		self.fig, self.ax = plt.subplots()
		self.line_meas, = self.ax.plot([], [], 'b-', label='Measured Position Y')
		self.line_target, = self.ax.plot([], [], 'r--', label='Desired Path (Target Y)') 
		self.ax.set_xlabel('Time (s)')
		self.ax.set_ylabel('Position Y (m)')
		self.ax.set_title(f'Path Comparison: {args.label}')
		self.ax.legend()
		self.ax.grid(True)

		print(f"---LOGGER:Recording started. Path Type: {args.path}. Saving to {self.filename} ---")
		# End of user custom code region. Please don't edit beyond this point.


	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiCanPythonGateway.initialize(dSession, self.componentId)
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				sim_time_s = vsiCommonPythonApi.getSimulationTimeInNs() / 1e9
				
				# --- Replicate Target Calculation for Plotting ---
				target_y = 0.0
				if self.args.path == 'sine':
					target_y = 0.5 * math.sin(2 * math.pi * 0.1 * sim_time_s)
				elif self.args.path == 'circle':
					radius = 20.0
					seg = 15.0
					x = self.mySignals.pos_x
					def get_arc_y(u, r):
						return r - math.sqrt(max(0.0, r**2 - u**2))
					h = get_arc_y(seg, radius)
					if x <= seg:
						target_y = get_arc_y(x, radius)
					elif x <= 2.0 * seg:
						target_y = h - get_arc_y(x - seg, radius)
					elif x <= 3.0 * seg:
						target_y = get_arc_y(x - 2.0 * seg, radius)
					else:
						target_y = h
				elif self.args.path == 'straight':
					target_y = 0.0

				self.writer.writerow([
					sim_time_s, self.mySignals.pos_x, self.mySignals.pos_y,
					self.mySignals.theta, self.mySignals.lateral_error, 
					self.mySignals.angular_w, target_y
				])

				# Update Histories
				self.time_history.append(sim_time_s)
				self.pos_y_history.append(self.mySignals.pos_y)
				self.target_y_history.append(target_y)
				self.counter += 1
				
				if self.counter % self.plot_decimation == 0:
					self.line_meas.set_data(self.time_history, self.pos_y_history)
					self.line_target.set_data(self.time_history, self.target_y_history)
					self.ax.relim()
					self.ax.autoscale_view()
					plt.pause(0.001) 
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 10)
				self.mySignals.pos_x, receivedData = self.unpackBytes('d', receivedData, self.mySignals.pos_x)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
				self.mySignals.pos_y, receivedData = self.unpackBytes('d', receivedData, self.mySignals.pos_y)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 12)
				self.mySignals.theta, receivedData = self.unpackBytes('d', receivedData, self.mySignals.theta)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 13)
				self.mySignals.angular_w, receivedData = self.unpackBytes('d', receivedData, self.mySignals.angular_w)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 18)
				self.mySignals.lateral_error, receivedData = self.unpackBytes('d', receivedData, self.mySignals.lateral_error)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 19)
				self.mySignals.theta_error, receivedData = self.unpackBytes('d', receivedData, self.mySignals.theta_error)

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal received.")
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

		plt.ioff()
		plt.show()

	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)

	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']: numBytes = 1
			elif signalType in ['h', 'H']: numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']: numBytes = 4
			elif signalType in ['q', 'Q', 'd']: numBytes = 8
			else: raise Exception('invalid signal type')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested() # Fixed: Added back 'Stop'
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()


def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost')

	# Start of user custom code region. Please apply edits only within these regions:  Main method
	inputArgs.add_argument('--label', type=str, default='straight_baseline', help='CSV filename')
	inputArgs.add_argument('--path', type=str, default='straight', choices=['straight', 'sine', 'circle'])
	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
	logger = Logger(args)
	logger.mainThread()


if __name__ == '__main__':
	main()
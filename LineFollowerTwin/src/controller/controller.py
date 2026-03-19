#!/usr/bin/env python3
from __future__ import print_function
import random
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.measured_x = 0
		self.measured_y = 0
		self.measured_theta = 0

		# Outputs
		self.linear_v = 0
		self.angular_w = 0
		self.lateral_error = 0
		self.theta_error = 0


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

# End of user custom code region. Please don't edit beyond this point.
class Controller:

	def __init__(self, args):
		self.componentId = 1
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50102
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor
		self.args=args
		self.kp=args.kp
		self.ki=args.ki
		self.kd=args.kd
		self.target_y=0.0

		#state variables for the "I" and "D" parts
		self.integral_error=0.0
		self.last_error=0.0
		self.dt=0.01

		# Rate Limiter variables
		self.prev_w = 0.0
		self.max_delta_w = 0.8

		# Filter for circular path noise reduction
		self.filtered_error = 0.0
		self.alpha = 0.2 
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
				sim_time_sec = vsiCommonPythonApi.getSimulationTimeInNs() / 1e9

				# --- Path Selection Logic ---
				if self.args.path == 'sine':
					self.target_y = 0.5 * math.sin(2 * math.pi * 0.1 * sim_time_sec)
				
				elif self.args.path == 'circle':
					radius = 20.0
					seg = 15.0
					x = self.mySignals.measured_x
					
					def get_arc_y(u, r):
						# Returns the y-offset for a circle segment of radius r
						return r - math.sqrt(max(0.0, r**2 - u**2))

					# Calculate height of a full segment once to use as an offset
					h = get_arc_y(seg, radius)

					if x <= seg:
						# First Arc (Curving Up)
						self.target_y = get_arc_y(x, radius)
					elif x <= 2.0 * seg:
						# Second Arc (Curving Down - Starting from peak 'h')
						self.target_y = h - get_arc_y(x - seg, radius)
					elif x <= 3.0 * seg:
						# Third Arc (Curving Up - Starting from '0' again)
						self.target_y = get_arc_y(x - 2.0 * seg, radius)
					else:
						# Final Segment Lock
						self.target_y = h
				
				else: # Default 'straight'
					self.target_y = 0.0
				
				# Error Calculation
				noise_amplitude = 0.01
				simulated_noise = random.uniform(-noise_amplitude, noise_amplitude)
				noisy_y = self.mySignals.measured_y + simulated_noise
				current_error = self.target_y - noisy_y

				# --- Apply Low-Pass Filter to all paths ---
				self.filtered_error = (self.alpha * current_error) + ((1 - self.alpha) * self.filtered_error)
				active_error = self.filtered_error

				# --- Path-Specific Integral Logic ---
				if self.args.path == 'circle':
					# Integral Anti-Windup (Tighter cap for circles)
					self.integral_error += active_error * self.dt
					self.integral_error = max(min(self.integral_error, 1.0), -1.0)
				else:
					# Standard logic for other paths
					if abs(active_error) <= 0.005:
						temp_error = 0.0
					else:
						temp_error = active_error
					self.integral_error += temp_error * self.dt
					self.integral_error = max(min(self.integral_error, 5.0), -5.0)

				self.mySignals.lateral_error = active_error

				# PID Calculations
				derivative = (active_error - self.last_error) / self.dt
				self.last_error = active_error

				# Rate Limiter Implementation
				raw_w = (self.kp * active_error) + (self.ki * self.integral_error) + (self.kd * derivative)
				target_w = max(min(raw_w, 3.0), -3.0)

				delta_w = target_w - self.prev_w
				
				# Faster steering allowed for circles to keep up with curvature
				current_max_delta = 1.2 if self.args.path == 'circle' else self.max_delta_w

				if delta_w > current_max_delta:
					delta_w = current_max_delta
				elif delta_w < -current_max_delta:
					delta_w = -current_max_delta

				self.mySignals.angular_w = self.prev_w + delta_w
				self.prev_w = self.mySignals.angular_w

				self.mySignals.linear_v = 0.5
				raw_theta_error = 0.0 - self.mySignals.measured_theta
				self.mySignals.theta_error = math.atan2(math.sin(raw_theta_error), math.cos(raw_theta_error))
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 14)
				self.mySignals.measured_x, receivedData = self.unpackBytes('d', receivedData, self.mySignals.measured_x)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 15)
				self.mySignals.measured_y, receivedData = self.unpackBytes('d', receivedData, self.mySignals.measured_y)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 17)
				self.mySignals.measured_theta, receivedData = self.unpackBytes('d', receivedData, self.mySignals.measured_theta)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				vsiCanPythonGateway.setCanId(13)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.angular_w), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(16)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.linear_v), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(18)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.lateral_error), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(19)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.theta_error), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user don't edit beyond this point.

				print("\n+=controller+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tmeasured_x =", end = " ")
				print(self.mySignals.measured_x)
				print("\tmeasured_y =", end = " ")
				print(self.mySignals.measured_y)
				print("\tmeasured_theta =", end = " ")
				print(self.mySignals.measured_theta)
				print("  Outputs:")
				print("\tlinear_v =", end = " ")
				print(self.mySignals.linear_v)
				print("\tangular_w =", end = " ")
				print(self.mySignals.angular_w)
				print("\tlateral_error =", end = " ")
				print(self.mySignals.lateral_error)
				print("\ttheta_error =", end = " ")
				print(self.mySignals.theta_error)
				print("\n\n")

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
				print("Terminate signal has been received from one of the VSI clients")
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)


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
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()


def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL')

	# Start of user custom code region. Please apply edits only within these regions:  Main method
	inputArgs.add_argument('--kp', type=float, default=4.0, help='Proportional Gain')
	inputArgs.add_argument('--ki',type=float,default=0.0, help='Integral Gain')
	inputArgs.add_argument('--kd',type=float,default=3.0,help='Derivative Gain')
	inputArgs.add_argument('--path',type=str,default='straight',choices=['straight','sine', 'circle'])
	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	controller = Controller(args)
	controller.mainThread()


if __name__ == '__main__':
    main()
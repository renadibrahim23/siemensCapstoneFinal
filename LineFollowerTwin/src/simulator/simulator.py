#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math
import random

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.linear_v = 0
		self.angular_w = 0

		# Outputs
		self.pos_x = 0
		self.pos_y = 0
		self.measured_x = 0
		self.measured_y = 0
		self.theta = 0
		self.measured_theta = 0




# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

# End of user custom code region. Please don't edit beyond this point.
class Simulator:

	def __init__(self, args):
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor
		self.mySignals.pos_x=0.0
		
		self.mySignals.theta=0.0
		self.dt=0.01

		if args.random_spawn:
			self.mySignals.pos_y=random.uniform(-0.5, 0.5)
			print(f"--RANDOM SPAWN: starting at Y= {self.mySignals.pos_y:.3f}---")
		else:
			self.mySignals.pos_y=0.5
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
				v=self.mySignals.linear_v
				w=self.mySignals.angular_w

				self.mySignals.theta += w * self.dt

				self.mySignals.pos_x += v * math.cos(self.mySignals.theta)*self.dt
				self.mySignals.pos_y += v * math.sin(self.mySignals.theta)*self.dt

				self.mySignals.measured_x=self.mySignals.pos_x
				self.mySignals.measured_y=self.mySignals.pos_y
				self.mySignals.measured_theta=self.mySignals.theta
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 13)
				self.mySignals.angular_w, receivedData = self.unpackBytes('d', receivedData, self.mySignals.angular_w)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 16)
				self.mySignals.linear_v, receivedData = self.unpackBytes('d', receivedData, self.mySignals.linear_v)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				vsiCanPythonGateway.setCanId(10)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.pos_x), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(11)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.pos_y), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(12)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.theta), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(14)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.measured_x), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(15)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.measured_y), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(17)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.measured_theta), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=simulator+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tlinear_v =", end = " ")
				print(self.mySignals.linear_v)
				print("\tangular_w =", end = " ")
				print(self.mySignals.angular_w)
				print("  Outputs:")
				print("\tpos_x =", end = " ")
				print(self.mySignals.pos_x)
				print("\tpos_y =", end = " ")
				print(self.mySignals.pos_y)
				print("\tmeasured_x =", end = " ")
				print(self.mySignals.measured_x)
				print("\tmeasured_y =", end = " ")
				print(self.mySignals.measured_y)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\tmeasured_theta =", end = " ")
				print(self.mySignals.measured_theta)
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
				# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
				# receive the terminate packet before terminating this client
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
			# receive the terminate packet before terminating this client
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)




		# Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function

		# End of user custom code region. Please don't edit beyond this point.



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
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

	# Start of user custom code region. Please apply edits only within these regions:  Main method
	inputArgs.add_argument('--random_spawn', action='store_true', help='spawn at a random Y offset')
	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	simulator = Simulator(args)
	simulator.mainThread()



if __name__ == '__main__':
    main()

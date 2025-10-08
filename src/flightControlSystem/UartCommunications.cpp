/*
BSD 3-Clause License

Copyright (c) 2025, Nishant Kumar, Jai Willems

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "UartCommunications.h"

template <typename TxType, typename RxType>
void UartCommunications<TxType, RxType>::setup(
	uint8_t rxPin,
	uint8_t txPin,
	unsigned long baudRate
) {
	_serial = new SoftwareSerial(
		rxPin,
		txPin
	);
	_serial->begin(baudRate);
}

template <typename TxType, typename RxType>
void UartCommunications<TxType, RxType>::transmit(
	TxType data
) {
	_serial->write(START_MARKER);
	_serial->write(
		(char*)&data,
		sizeof(data)
	);
}

template <typename TxType, typename RxType>
bool UartCommunications<TxType, RxType>::available() {
	return _serial->available() > sizeof(_rxDataBuffer) + 1;
}

template <typename TxType, typename RxType>
RxType UartCommunications<TxType, RxType>::receive() {
	byte* structStart = reinterpret_cast<byte*>(&_rxDataBuffer);

	byte data = _serial->read();

	if (data == START_MARKER) {

		for (byte n = 0; n < sizeof(_rxDataBuffer); n++) {
			*(structStart + n) = _serial->read();
		}
		while (_serial->available() > 0) {
			byte dumpTheData = _serial->read();
		}
	}

	return _rxDataBuffer;
}

template class UartCommunications<DroneState, FlightInputs>;

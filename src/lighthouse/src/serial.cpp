#include "lighthouse_track/serial.h"
#include <iostream>
#include <vector>
using namespace std;

#ifdef __unix

UsbSerialLinux::UsbSerialLinux() {
  device_ = "";
  fd_ = 0;
}

UsbSerialLinux::UsbSerialLinux(string device) {
  device_ = "";
  fd_ = 0;
  Open(device);
}

int UsbSerialLinux::Open(string device) {
  if (IsOpened()) {
    std::cout << "device is opened" << std::endl;
    return -1;
  }
  device_ = device;
  fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    cerr << "Can't Open Serial Port" << endl;
    return -1;
  }
  struct termios options;
  if (tcgetattr(fd_, &options) != 0)
    cout << "SetupSerial 1" << endl;;
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	if (tcsetattr(fd_, TCSANOW, &options) != 0)
    cout << "SetupSerial 3" << endl;
  return fd_;
}

bool UsbSerialLinux::Read(char *buf, int length) {
  if (!IsOpened()) {
    cout << "error read serial: is not opened" << endl;
    return false;
  }
  char* buf_tmp = buf;
  int bytes_remain = length;
  int bytes_read = 0;
  int count = 0;
  do {
    bytes_read = read(fd_, buf_tmp, bytes_remain);
    if (bytes_read > 0) {
      count = 0;
      buf_tmp += bytes_read;
      bytes_remain -= bytes_read;
    }
    else {
      count++;
      usleep(10000);
      if (count > 500) {
        cout << "error read serial: timeout" << endl;
        return false;
      }
    }
  } while(bytes_remain > 0);
  return true;
}

bool UsbSerialLinux::Close() {
  if (IsOpened())
    std::cout << "Serial Closed" << std::endl;
    return close(fd_);
}

#elif defined(_WIN32)

UsbSerialWin::UsbSerialWin() {
	device_ = "";
	h_comm_ = nullptr;
	memset(&over_lapped_, 0, sizeof(over_lapped_));
}

UsbSerialWin::UsbSerialWin(string device) {
	h_comm_ = nullptr;
	if (Open(device) != 0) {
		cerr << "Can't open serial port" << endl;
	}
	memset(&over_lapped_, 0, sizeof(over_lapped_));
}

int UsbSerialWin::Open(string device) {
	if (IsOpened()) {
		std::cout << "Device has been opened" << std::endl;
		return -1;
	}
	device_ = device;
	h_comm_ = CreateFile(LPCTSTR(device_.c_str()), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
	if (h_comm_ == INVALID_HANDLE_VALUE) {
		cerr << "Can't open serial port" << endl;
		return -1;
	}

	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	if (!GetCommState(h_comm_, &dcb)) {
		cerr << "Set DCB failed" << endl;
		return -1;
	}
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = 115200;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	dcb.fErrorChar = 0;
	dcb.fBinary = 1;
	dcb.fNull = 0;
	dcb.fAbortOnError = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 2;
	dcb.XoffLim = 4;
	dcb.XonChar = 0x13;
	dcb.XoffChar = 0x19;
	dcb.EvtChar = 0;

	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	if (!SetCommTimeouts(h_comm_, &timeouts)) {
		cerr << "Set timeOut failed" << endl;
		return -1;
	}

	PurgeComm(h_comm_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	return 0;
}

bool UsbSerialWin::Read(char *buf, int length) {
	if (!IsOpened()) {
		cout << "Serial is not opened" << endl;
		return false;
	}

	char* buf_tmp = buf;
	unsigned long bytes_remain = length;
	unsigned long dw_error = 0;
	unsigned long bytes_read = 0;
	unsigned short count = 0;

	COMSTAT comstat;

	do {

		bool b_result = ClearCommError(h_comm_, &dw_error, &comstat);

		if (comstat.cbInQue == 0)
			return -1;
		b_result = ReadFile(h_comm_,  //通信设备（此处为串口）句柄，由CreateFile()返回值得到  
			buf,											//指向接收缓冲区  
			bytes_remain,							//指明要从串口中读取的字节数  
			&bytes_read,
			&over_lapped_);						//OVERLAPPED结构

		bool b_read = TRUE;
		if (!b_result)
		{
			switch (dw_error == GetLastError())
			{
			case ERROR_IO_PENDING:
				b_read = FALSE;
				break;
			default:
				break;
			}
		}

		if (!b_read)
		{
			b_read = TRUE;
			b_result = GetOverlappedResult(h_comm_, &over_lapped_, &bytes_read, TRUE);
		}
		if (bytes_read > 0) {
			count = 0;
			buf_tmp += bytes_read;
			bytes_remain -= bytes_read;
		}
		else {
			count++;
			Sleep(1);
			if (count > 500) {
				cout << "Error read serial: timeout" << endl;
				return false;
			}
		}
	} while (bytes_remain > 0);
	return true;
}

bool UsbSerialWin::Close() {
	if (IsOpened()) {
		CloseHandle(h_comm_);
		return true;
	}
	return false;
}
#endif

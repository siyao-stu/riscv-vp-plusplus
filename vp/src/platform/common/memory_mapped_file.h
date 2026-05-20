#pragma once

#include <stdint.h>
#include <tlm_utils/simple_target_socket.h>
#include <unistd.h>  //truncate

#include <array>
#include <fstream>  //file IO
#include <iomanip>
#include <iostream>
#include <vector>
#include <systemc>

#include "platform/common/bus.h"
#include "util/propertytree.h"

using namespace std;
using namespace sc_core;
using namespace tlm_utils;

struct MemoryMappedFile : public sc_core::sc_module {
	/* config properties */
	sc_core::sc_time prop_clock_cycle_period = sc_core::sc_time(10, sc_core::SC_NS);
	unsigned int prop_access_clock_cycles = 3;

	sc_core::sc_time access_delay_base;

	simple_target_socket<MemoryMappedFile> tsock;

	string mFilepath;
	uint32_t mSize;
	fstream file;
	bool emulate_cfi_nor = false;
	uint32_t cfi_block_size = 256u * 1024u;

	enum class CfiMode { ReadArray, ReadStatus, ReadDeviceId, ReadCfiQuery };
	enum class CfiProgramState { None, WordProgramPendingData, BlockErasePendingConfirm, BufferedPendingCount, BufferedPendingData, BufferedPendingConfirm };

	CfiMode cfi_mode = CfiMode::ReadArray;
	CfiProgramState cfi_prog_state = CfiProgramState::None;
	uint32_t cfi_status = 0x00800080;  // P30_SR_BIT_WRITE for both 16-bit chips
	unsigned cfi_program_addr = 0;
	unsigned cfi_buffer_base_addr = 0;
	unsigned cfi_buffer_expected_words = 0;
	unsigned cfi_buffer_received_words = 0;
	std::vector<uint32_t> cfi_buffer_words;

	static bool is_dual_cmd(uint32_t value) {
		return (value & 0xFFFFu) == ((value >> 16) & 0xFFFFu);
	}

	static uint16_t cmd16(uint32_t value) {
		return static_cast<uint16_t>(value & 0xFFFFu);
	}

	bool read_word(unsigned addr, uint32_t &value) {
		std::array<uint8_t, 4> bytes = {0xFF, 0xFF, 0xFF, 0xFF};
		if (addr + bytes.size() > mSize) {
			return false;
		}
		read_data(addr, bytes.data(), bytes.size());
		value = static_cast<uint32_t>(bytes[0]) |
		        (static_cast<uint32_t>(bytes[1]) << 8) |
		        (static_cast<uint32_t>(bytes[2]) << 16) |
		        (static_cast<uint32_t>(bytes[3]) << 24);
		return true;
	}

	void program_word(unsigned addr, uint32_t data) {
		if (addr + 4 > mSize) {
			return;
		}
		uint32_t old = 0xFFFFFFFFu;
		read_word(addr, old);
		uint32_t programmed = old & data;
		std::array<uint8_t, 4> out = {
		    static_cast<uint8_t>(programmed & 0xFFu),
		    static_cast<uint8_t>((programmed >> 8) & 0xFFu),
		    static_cast<uint8_t>((programmed >> 16) & 0xFFu),
		    static_cast<uint8_t>((programmed >> 24) & 0xFFu),
		};
		write_data(addr, out.data(), out.size());
	}

	bool cfi_read_returns_status() const {
		return cfi_mode == CfiMode::ReadStatus ||
		       cfi_prog_state == CfiProgramState::BufferedPendingCount ||
		       cfi_prog_state == CfiProgramState::BufferedPendingConfirm ||
		       cfi_prog_state == CfiProgramState::BlockErasePendingConfirm;
	}

	void erase_block_containing(unsigned addr) {
		if (cfi_block_size == 0) {
			return;
		}
		unsigned base = (addr / cfi_block_size) * cfi_block_size;
		if (base >= mSize) {
			return;
		}
		unsigned len = std::min<unsigned>(cfi_block_size, mSize - base);
		std::vector<uint8_t> erased(len, 0xFF);
		write_data(base, erased.data(), len);
	}

	bool handle_cfi_write(unsigned addr, uint8_t *src, unsigned len) {
		if (!emulate_cfi_nor || len != 4) {
			return false;
		}

		uint32_t value = static_cast<uint32_t>(src[0]) |
		                 (static_cast<uint32_t>(src[1]) << 8) |
		                 (static_cast<uint32_t>(src[2]) << 16) |
		                 (static_cast<uint32_t>(src[3]) << 24);

		if (cfi_prog_state == CfiProgramState::WordProgramPendingData) {
			program_word(addr, value);
			cfi_status = 0x00800080;
			cfi_prog_state = CfiProgramState::None;
			return true;
		}

		if (cfi_prog_state == CfiProgramState::BufferedPendingData) {
			cfi_buffer_words.push_back(value);
			cfi_buffer_received_words++;
			if (cfi_buffer_received_words >= cfi_buffer_expected_words) {
				cfi_prog_state = CfiProgramState::BufferedPendingConfirm;
			}
			return true;
		}

		if (!is_dual_cmd(value)) {
			return false;
		}

		uint16_t cmd = cmd16(value);
		switch (cmd) {
			case 0x00FF:  // READ ARRAY
				cfi_mode = CfiMode::ReadArray;
				cfi_status = 0x00800080;
				cfi_prog_state = CfiProgramState::None;
				cfi_buffer_words.clear();
				return true;
			case 0x0070:  // READ STATUS REGISTER
				cfi_mode = CfiMode::ReadStatus;
				return true;
			case 0x0050:  // CLEAR STATUS REGISTER
				cfi_status = 0x00800080;
				return true;
			case 0x0090:  // READ DEVICE ID
				cfi_mode = CfiMode::ReadDeviceId;
				return true;
			case 0x0098:  // READ CFI QUERY
				cfi_mode = CfiMode::ReadCfiQuery;
				return true;
			case 0x0040:  // WORD PROGRAM SETUP
			case 0x0010:  // ALT WORD PROGRAM SETUP
				cfi_program_addr = addr;
				cfi_status = 0x00800080;
				cfi_prog_state = CfiProgramState::WordProgramPendingData;
				return true;
			case 0x0020:  // BLOCK ERASE SETUP
				cfi_program_addr = addr;
				cfi_status = 0x00800080;
				cfi_prog_state = CfiProgramState::BlockErasePendingConfirm;
				return true;
			case 0x00E8:  // BUFFERED PROGRAM SETUP
				cfi_buffer_base_addr = addr;
				cfi_buffer_expected_words = 0;
				cfi_buffer_received_words = 0;
				cfi_buffer_words.clear();
				cfi_status = 0x00800080;
				cfi_prog_state = CfiProgramState::BufferedPendingCount;
				return true;
			case 0x0060:  // LOCK SETUP
				return true;
			case 0x00D0:  // UNLOCK / ERASE CONFIRM / BUFFERED CONFIRM
				if (cfi_prog_state == CfiProgramState::BlockErasePendingConfirm) {
					erase_block_containing(cfi_program_addr);
					cfi_status = 0x00800080;
					cfi_prog_state = CfiProgramState::None;
					return true;
				}
				if (cfi_prog_state == CfiProgramState::BufferedPendingConfirm) {
					for (unsigned i = 0; i < cfi_buffer_words.size(); i++) {
						program_word(cfi_buffer_base_addr + 4u * i, cfi_buffer_words[i]);
					}
					cfi_status = 0x00800080;
					cfi_prog_state = CfiProgramState::None;
					cfi_buffer_words.clear();
					return true;
				}
				return true;
			default:
				if (cfi_prog_state == CfiProgramState::BufferedPendingCount) {
					cfi_buffer_expected_words = static_cast<unsigned>(cmd) + 1u;
					cfi_buffer_received_words = 0;
					cfi_status = 0x00800080;
					cfi_prog_state = CfiProgramState::BufferedPendingData;
					return true;
				}
				return false;
		}
	}

	bool handle_cfi_read(unsigned addr, uint8_t *dst, unsigned len) {
		if (!emulate_cfi_nor || len != 4) {
			return false;
		}

		if (cfi_read_returns_status()) {
			dst[0] = static_cast<uint8_t>(cfi_status & 0xFFu);
			dst[1] = static_cast<uint8_t>((cfi_status >> 8) & 0xFFu);
			dst[2] = static_cast<uint8_t>((cfi_status >> 16) & 0xFFu);
			dst[3] = static_cast<uint8_t>((cfi_status >> 24) & 0xFFu);
			return true;
		}

		if (cfi_mode == CfiMode::ReadDeviceId) {
			// Report unlocked / no-lockdown bits for both chips.
			dst[0] = 0;
			dst[1] = 0;
			dst[2] = 0;
			dst[3] = 0;
			return true;
		}

		if (cfi_mode == CfiMode::ReadCfiQuery) {
			// Minimal CFI emulation: expose zeros, while read-array contains real data.
			dst[0] = 0;
			dst[1] = 0;
			dst[2] = 0;
			dst[3] = 0;
			return true;
		}

		return false;
	}

	MemoryMappedFile(sc_module_name, string &filepath, uint32_t size, bool enable_cfi_nor = false)
	    : mFilepath(filepath), mSize(size), emulate_cfi_nor(enable_cfi_nor) {
		/* get config properties from global property tree (or use default) */
		VPPP_PROPERTY_GET("MemoryMappedFile." + name(), "clock_cycle_period", sc_core::sc_time,
		                  prop_clock_cycle_period);
		VPPP_PROPERTY_GET("MemoryMappedFile." + name(), "access_clock_cycles", uint64_t, prop_access_clock_cycles);
		uint64_t cfi_block_size_prop = cfi_block_size;
		VPPP_PROPERTY_GET("MemoryMappedFile." + name(), "cfi_block_size", uint64_t, cfi_block_size_prop);
		cfi_block_size = static_cast<uint32_t>(cfi_block_size_prop);

		access_delay_base = prop_access_clock_cycles * prop_clock_cycle_period;

		tsock.register_b_transport(this, &MemoryMappedFile::transport);

		if (filepath.size() == 0 || size == 0) {  // no file
			return;
		}
		file.open(mFilepath, ofstream::in | ofstream::out | ofstream::binary);
		if (!file.is_open() || !file.good()) {
			// cerr << "Failed to open " << mFilepath << ": " << strerror(errno)
			// << endl;
			file.open(mFilepath, ofstream::in | ofstream::out | ofstream::binary | ios_base::trunc);
		}
		int stat = truncate(mFilepath.c_str(), mSize);
		assert(stat == 0 && "truncate failed");
		assert(file.is_open() && file.good() && "File could not be opened");
	}

	~MemoryMappedFile() {
		file.close();
	}

	void write_data(unsigned addr, uint8_t *src, unsigned num_bytes) {
		assert(addr + num_bytes <= mSize);
		if (!file.is_open()) {
			return;
		}
		file.seekp(addr, file.beg);
		if (!file.fail()) {
			file.write(reinterpret_cast<char *>(src), num_bytes);
			file.flush();
		}
		if (file.fail()) {
			cerr << name() << ": ERROR: Failed to write to \"" << mFilepath << "\"!" << endl;
			file.clear();
		}
	}

	void read_data(unsigned addr, uint8_t *dst, unsigned num_bytes) {
		assert(addr + num_bytes <= mSize);
		if (!file.is_open()) {
			memset(dst, 0, num_bytes);
			return;
		}
		file.seekg(addr, file.beg);
		if (!file.fail()) {
			file.read(reinterpret_cast<char *>(dst), num_bytes);
		}
		if (file.fail()) {
			cerr << name() << ": ERROR: Failed to read from \"" << mFilepath << "\"!" << endl;
			file.clear();
			memset(dst, 0, num_bytes);
		}
	}

	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		tlm::tlm_command cmd = trans.get_command();
		unsigned addr = trans.get_address();
		auto *ptr = trans.get_data_ptr();
		auto len = trans.get_data_length();

		assert(addr < mSize);

		if (cmd == tlm::TLM_WRITE_COMMAND) {
			if (!handle_cfi_write(addr, ptr, len)) {
				write_data(addr, ptr, len);
			}
		} else if (cmd == tlm::TLM_READ_COMMAND) {
			if (std::string(name()) == "flash1") {
				// constexpr uint64_t kFlash1BaseAddr = 0x22000000ull;
				// constexpr uint64_t kTailRangeStart = 0x2203f000ull;
				// constexpr uint64_t kTailRangeEnd = 0x22040000ull;
				// uint64_t req_start = kFlash1BaseAddr + addr;
				// uint64_t req_end = req_start + len;
				// if (req_start < kTailRangeEnd && req_end > kTailRangeStart) {
					// cerr << "[flash1-tail-read] global=0x" << hex << req_start << "..0x" << req_end
					//      << " local=0x" << addr << " len=" << dec << len << endl;
				// }
			}
			if (!handle_cfi_read(addr, ptr, len)) {
				read_data(addr, ptr, len);
			}
		} else {
			sc_assert(false && "unsupported tlm command");
		}

		delay += len * access_delay_base;
	}
};

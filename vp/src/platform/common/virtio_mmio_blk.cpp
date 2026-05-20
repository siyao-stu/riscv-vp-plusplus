#include "virtio_mmio_blk.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>


namespace {
constexpr uint32_t kVirtioMmioMagic = 0x74726976;  // "virt"
constexpr uint32_t kVirtioMmioVersion = 2;         // virtio 1.0
constexpr uint32_t kVirtioDeviceIdBlk = 2;
constexpr uint32_t kVirtioVendorId = 0x1af4;

constexpr uint32_t kVirtioMmioOffsetMagic = 0x00;
constexpr uint32_t kVirtioMmioOffsetVersion = 0x04;
constexpr uint32_t kVirtioMmioOffsetDeviceId = 0x08;
constexpr uint32_t kVirtioMmioOffsetVendorId = 0x0c;
constexpr uint32_t kVirtioMmioOffsetHostFeatures = 0x10;
constexpr uint32_t kVirtioMmioOffsetHostFeaturesSel = 0x14;
constexpr uint32_t kVirtioMmioOffsetGuestFeatures = 0x20;
constexpr uint32_t kVirtioMmioOffsetGuestFeaturesSel = 0x24;
constexpr uint32_t kVirtioMmioOffsetGuestPageSize = 0x28;
constexpr uint32_t kVirtioMmioOffsetQueueSel = 0x30;
constexpr uint32_t kVirtioMmioOffsetQueueNumMax = 0x34;
constexpr uint32_t kVirtioMmioOffsetQueueNum = 0x38;
constexpr uint32_t kVirtioMmioOffsetQueueAlign = 0x3c;
constexpr uint32_t kVirtioMmioOffsetQueuePfn = 0x40;
constexpr uint32_t kVirtioMmioOffsetQueueReady = 0x44;
constexpr uint32_t kVirtioMmioOffsetQueueNotify = 0x50;
constexpr uint32_t kVirtioMmioOffsetInterruptStatus = 0x60;
constexpr uint32_t kVirtioMmioOffsetInterruptAck = 0x64;
constexpr uint32_t kVirtioMmioOffsetStatus = 0x70;
constexpr uint32_t kVirtioMmioOffsetQueueDescLo = 0x80;
constexpr uint32_t kVirtioMmioOffsetQueueDescHi = 0x84;
constexpr uint32_t kVirtioMmioOffsetQueueAvailLo = 0x90;
constexpr uint32_t kVirtioMmioOffsetQueueAvailHi = 0x94;
constexpr uint32_t kVirtioMmioOffsetQueueUsedLo = 0xa0;
constexpr uint32_t kVirtioMmioOffsetQueueUsedHi = 0xa4;
constexpr uint32_t kVirtioMmioOffsetConfigGeneration = 0xfc;

constexpr uint32_t kVirtioMmioDeviceConfigOffset = 0x100;

constexpr uint32_t kVringDescSize = 16;
constexpr uint16_t kVringDescFlagNext = 0x01;
constexpr uint16_t kVringDescFlagWrite = 0x02;

constexpr uint16_t kVringAvailNoInterrupt = 0x01;

constexpr uint32_t kVirtioBlkTypeIn = 0x00000000;
constexpr uint32_t kVirtioBlkTypeOut = 0x00000001;
constexpr uint32_t kVirtioBlkTypeFlush = 0x00000004;

constexpr uint8_t kVirtioBlkStatusOk = 0x00;
constexpr uint8_t kVirtioBlkStatusIoErr = 0x01;
constexpr uint8_t kVirtioBlkStatusUnsupp = 0x02;

constexpr uint64_t kVirtioFeatureVersion1 = 1ULL << 32;
constexpr uint64_t kVirtioBlkFeatureBlkSize = 1ULL << 6;
constexpr uint64_t kVirtioBlkFeatureRo = 1ULL << 5;

constexpr uint32_t kVirtioStatusAcknowledge = 1U << 0;
constexpr uint32_t kVirtioStatusDriver = 1U << 1;
constexpr uint32_t kVirtioStatusDriverOk = 1U << 2;
constexpr uint32_t kVirtioStatusFeaturesOk = 1U << 3;

constexpr uint32_t kVirtioMmioInterruptUsed = 0x1;

struct VirtioBlkReq {
	uint32_t type;
	uint32_t io_prio;
	uint64_t sector;
};

struct VringDesc {
	uint64_t addr;
	uint32_t len;
	uint16_t flags;
	uint16_t next;
};

struct VirtioBlkConfig {
	uint64_t capacity;
	uint32_t size_max;
	uint32_t seg_max;
	uint16_t cylinders;
	uint8_t heads;
	uint8_t sectors;
	uint32_t blk_size;
	uint8_t topology_physical_block_exp;
	uint8_t topology_alignment_offset;
	uint16_t topology_min_io_size;
	uint32_t topology_opt_io_size;
};

constexpr uint32_t kVirtioBlkConfigSize = sizeof(VirtioBlkConfig);
}  // namespace

VirtioMmioBlk::VirtioMmioBlk(sc_core::sc_module_name name, uint32_t irq_number) : irq_number(irq_number) {
	tsock.register_b_transport(this, &VirtioMmioBlk::transport);

	ext = new tlm_ext_initiator(this);
	trans.set_extension<tlm_ext_initiator>(ext);

	reset_device();
}

void VirtioMmioBlk::set_debug(bool enable) {
	debug = enable;
}

bool VirtioMmioBlk::insert_image(const std::string &path, bool read_only) {
	eject_image();
	image_read_only = read_only;
	image_path = path;

	int flags = read_only ? O_RDONLY : O_RDWR;
	image_fd = ::open(path.c_str(), flags | O_CLOEXEC);
	if (image_fd < 0 && !read_only) {
		image_fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
		if (image_fd >= 0) {
			image_read_only = true;
		}
	}
	if (image_fd < 0) {
		std::cerr << "[virtio-mmio-blk] failed to open image: " << path << std::endl;
		image_path.clear();
		return false;
	}

	update_capacity_from_file();
	reset_device();
	if (debug) {
		std::cout << "[virtio-mmio-blk] image " << path << " size=" << std::dec << image_size_bytes << " bytes"
		          << (image_read_only ? " (ro)" : "") << std::endl;
	}
	return true;
}

void VirtioMmioBlk::eject_image() {
	if (image_fd >= 0) {
		::close(image_fd);
	}
	image_fd = -1;
	image_size_bytes = 0;
	image_path.clear();
	reset_device();
}

void VirtioMmioBlk::halt() {
	std::cerr << "[VP] cannot halt initiator " << name() << std::endl;
}

std::string VirtioMmioBlk::name() {
	return sc_core::sc_module::name();
}

void VirtioMmioBlk::transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
	auto addr = trans.get_address();
	auto len = trans.get_data_length();
	auto *ptr = trans.get_data_ptr();

	if (trans.get_command() == tlm::TLM_READ_COMMAND) {
		if (addr >= kVirtioMmioDeviceConfigOffset) {
			if (!read_config(addr - kVirtioMmioDeviceConfigOffset, ptr, len)) {
				std::memset(ptr, 0, len);
			}
		} else {
			uint32_t value = read_reg(addr);
			std::memset(ptr, 0, len);
			std::memcpy(ptr, &value, std::min<size_t>(len, sizeof(value)));
		}
	} else if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
		if (addr >= kVirtioMmioDeviceConfigOffset) {
			write_config(addr - kVirtioMmioDeviceConfigOffset, ptr, len);
		} else {
			uint32_t value = 0;
			std::memcpy(&value, ptr, std::min<size_t>(len, sizeof(value)));
			write_reg(addr, value);
		}
	} else {
		throw std::runtime_error("unsupported TLM command in virtio-mmio-blk");
	}

	(void)delay;
}

uint32_t VirtioMmioBlk::read_reg(uint64_t offset) const {
	switch (offset) {
		case kVirtioMmioOffsetMagic:
			return kVirtioMmioMagic;
		case kVirtioMmioOffsetVersion:
			return kVirtioMmioVersion;
		case kVirtioMmioOffsetDeviceId:
			return image_fd >= 0 ? kVirtioDeviceIdBlk : 0;
		case kVirtioMmioOffsetVendorId:
			return kVirtioVendorId;
		case kVirtioMmioOffsetHostFeatures:
			return host_features_sel == 0 ? static_cast<uint32_t>(host_features)
			                             : static_cast<uint32_t>(host_features >> 32);
		case kVirtioMmioOffsetHostFeaturesSel:
			return host_features_sel;
		case kVirtioMmioOffsetGuestFeatures:
			return guest_features_sel == 0 ? static_cast<uint32_t>(guest_features)
			                              : static_cast<uint32_t>(guest_features >> 32);
		case kVirtioMmioOffsetGuestFeaturesSel:
			return guest_features_sel;
		case kVirtioMmioOffsetGuestPageSize:
			return guest_page_size;
		case kVirtioMmioOffsetQueueSel:
			return queue_sel;
		case kVirtioMmioOffsetQueueNumMax:
			return queue_sel == 0 ? queue_num_max : 0;
		case kVirtioMmioOffsetQueueNum:
			return queue_num;
		case kVirtioMmioOffsetQueueAlign:
			return queue_align;
		case kVirtioMmioOffsetQueuePfn:
			return 0;
		case kVirtioMmioOffsetQueueReady:
			return queue_ready;
		case kVirtioMmioOffsetInterruptStatus:
			return interrupt_status;
		case kVirtioMmioOffsetStatus:
			return device_status;
		case kVirtioMmioOffsetQueueDescLo:
			return static_cast<uint32_t>(queue_desc);
		case kVirtioMmioOffsetQueueDescHi:
			return static_cast<uint32_t>(queue_desc >> 32);
		case kVirtioMmioOffsetQueueAvailLo:
			return static_cast<uint32_t>(queue_avail);
		case kVirtioMmioOffsetQueueAvailHi:
			return static_cast<uint32_t>(queue_avail >> 32);
		case kVirtioMmioOffsetQueueUsedLo:
			return static_cast<uint32_t>(queue_used);
		case kVirtioMmioOffsetQueueUsedHi:
			return static_cast<uint32_t>(queue_used >> 32);
		case kVirtioMmioOffsetConfigGeneration:
			return 0;
		default:
			return 0;
	}
}

void VirtioMmioBlk::write_reg(uint64_t offset, uint32_t value) {
	switch (offset) {
		case kVirtioMmioOffsetHostFeaturesSel:
			host_features_sel = value;
			break;
		case kVirtioMmioOffsetGuestFeaturesSel:
			guest_features_sel = value;
			break;
		case kVirtioMmioOffsetGuestFeatures:
			if (guest_features_sel == 0) {
				guest_features = (guest_features & 0xffffffff00000000ULL) | value;
			} else {
				guest_features = (guest_features & 0x00000000ffffffffULL) | (static_cast<uint64_t>(value) << 32);
			}
			break;
		case kVirtioMmioOffsetGuestPageSize:
			guest_page_size = value;
			break;
		case kVirtioMmioOffsetQueueSel:
			queue_sel = value;
			break;
		case kVirtioMmioOffsetQueueNum:
			queue_num = static_cast<uint16_t>(value);
			break;
		case kVirtioMmioOffsetQueueAlign:
			queue_align = value;
			break;
		case kVirtioMmioOffsetQueueReady:
			queue_ready = value ? 1 : 0;
			if (debug) {
				std::cout << "[virtio-mmio-blk] queue_ready=" << queue_ready << std::endl;
			}
			break;
		case kVirtioMmioOffsetQueueDescLo:
			queue_desc = (queue_desc & 0xffffffff00000000ULL) | value;
			break;
		case kVirtioMmioOffsetQueueDescHi:
			queue_desc = (queue_desc & 0x00000000ffffffffULL) | (static_cast<uint64_t>(value) << 32);
			break;
		case kVirtioMmioOffsetQueueAvailLo:
			queue_avail = (queue_avail & 0xffffffff00000000ULL) | value;
			break;
		case kVirtioMmioOffsetQueueAvailHi:
			queue_avail = (queue_avail & 0x00000000ffffffffULL) | (static_cast<uint64_t>(value) << 32);
			break;
		case kVirtioMmioOffsetQueueUsedLo:
			queue_used = (queue_used & 0xffffffff00000000ULL) | value;
			break;
		case kVirtioMmioOffsetQueueUsedHi:
			queue_used = (queue_used & 0x00000000ffffffffULL) | (static_cast<uint64_t>(value) << 32);
			break;
		case kVirtioMmioOffsetQueueNotify:
			if (debug) {
				std::cout << "[virtio-mmio-blk] queue_notify=" << value << std::endl;
			}
			handle_queue_notify(value);
			break;
		case kVirtioMmioOffsetInterruptAck:
			interrupt_status &= ~value;
			break;
		case kVirtioMmioOffsetStatus: {
			if (value == 0) {
				reset_device();
				break;
			}

			uint32_t next_status = value;
			if ((value & kVirtioStatusFeaturesOk) != 0) {
				if ((guest_features & ~host_features) != 0) {
					next_status &= ~kVirtioStatusFeaturesOk;
				}
			}
			device_status = next_status;
			if (debug) {
				std::cout << "[virtio-mmio-blk] status=0x" << std::hex << device_status << std::dec << std::endl;
			}
			break;
		}
		default:
			break;
	}
}

bool VirtioMmioBlk::read_config(uint64_t offset, uint8_t *data, size_t len) const {
	if (offset + len > kVirtioBlkConfigSize) {
		return false;
	}

	VirtioBlkConfig cfg{};
	cfg.capacity = image_size_bytes / 512;
	cfg.blk_size = 512;

	std::memcpy(data, reinterpret_cast<const uint8_t *>(&cfg) + offset, len);
	return true;
}

bool VirtioMmioBlk::write_config(uint64_t offset, const uint8_t *data, size_t len) {
	(void)offset;
	(void)data;
	(void)len;
	return false;
}

bool VirtioMmioBlk::read_mem(uint64_t addr, uint8_t *data, size_t len) {
	sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
	trans.set_command(tlm::TLM_READ_COMMAND);
	trans.set_address(addr);
	trans.set_data_ptr(data);
	trans.set_data_length(len);

	isock->b_transport(trans, delay);
	if (delay != sc_core::SC_ZERO_TIME) {
		sc_core::wait(delay);
	}

	return trans.get_response_status() == tlm::TLM_OK_RESPONSE;
}

bool VirtioMmioBlk::write_mem(uint64_t addr, const uint8_t *data, size_t len) {
	sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
	trans.set_command(tlm::TLM_WRITE_COMMAND);
	trans.set_address(addr);
	trans.set_data_ptr(const_cast<uint8_t *>(data));
	trans.set_data_length(len);

	isock->b_transport(trans, delay);
	if (delay != sc_core::SC_ZERO_TIME) {
		sc_core::wait(delay);
	}

	return trans.get_response_status() == tlm::TLM_OK_RESPONSE;
}

void VirtioMmioBlk::handle_queue_notify(uint32_t queue_id) {
	if (queue_id != 0) {
		return;
	}
	process_available();
}

void VirtioMmioBlk::process_available() {
	if (queue_ready == 0 || queue_num == 0 || queue_desc == 0 || queue_avail == 0 || queue_used == 0) {
		return;
	}

	uint8_t avail_hdr[4] = {};
	if (!read_mem(queue_avail, avail_hdr, sizeof(avail_hdr))) {
		return;
	}
	uint16_t avail_flags = read_u16_le(avail_hdr);
	uint16_t avail_idx = read_u16_le(avail_hdr + 2);
	if (debug) {
		std::cout << "[virtio-mmio-blk] avail idx=" << avail_idx << " last=" << last_avail_idx << std::endl;
	}

	while (last_avail_idx != avail_idx) {
		uint16_t ring_idx = last_avail_idx % queue_num;
		uint8_t ring_entry[2] = {};
		uint64_t ring_addr = queue_avail + 4 + static_cast<uint64_t>(ring_idx) * 2;
		if (!read_mem(ring_addr, ring_entry, sizeof(ring_entry))) {
			return;
		}
		uint16_t head = read_u16_le(ring_entry);
		process_descriptor_chain(head);
		last_avail_idx++;
	}

	if ((avail_flags & kVringAvailNoInterrupt) == 0) {
		interrupt_status |= kVirtioMmioInterruptUsed;
		trigger_interrupt();
	}
}

void VirtioMmioBlk::process_descriptor_chain(uint16_t head_idx) {
	if (image_fd < 0) {
		return;
	}

	uint8_t desc_buf[kVringDescSize] = {};
	uint64_t desc_addr = queue_desc + static_cast<uint64_t>(head_idx) * kVringDescSize;
	if (!read_mem(desc_addr, desc_buf, sizeof(desc_buf))) {
		return;
	}

	VringDesc desc0{};
	desc0.addr = read_u64_le(desc_buf);
	desc0.len = read_u32_le(desc_buf + 8);
	desc0.flags = read_u16_le(desc_buf + 12);
	desc0.next = read_u16_le(desc_buf + 14);

	VirtioBlkReq req{};
	if (desc0.len < sizeof(req) || (desc0.flags & kVringDescFlagNext) == 0) {
		return;
	}
	if (!read_mem(desc0.addr, reinterpret_cast<uint8_t *>(&req), sizeof(req))) {
		return;
	}
	if (debug) {
		std::cout << "[virtio-mmio-blk] req type=0x" << std::hex << req.type << " sector=0x" << req.sector
		          << std::dec << " head=" << head_idx << std::endl;
	}

	uint16_t data_desc_idx = 0;
	uint16_t status_desc_idx = 0;
	uint32_t data_len = 0;
	bool has_data = false;

	uint8_t desc1_buf[kVringDescSize] = {};
	uint64_t desc1_addr = queue_desc + static_cast<uint64_t>(desc0.next) * kVringDescSize;
	if (!read_mem(desc1_addr, desc1_buf, sizeof(desc1_buf))) {
		return;
	}

	VringDesc desc1{};
	desc1.addr = read_u64_le(desc1_buf);
	desc1.len = read_u32_le(desc1_buf + 8);
	desc1.flags = read_u16_le(desc1_buf + 12);
	desc1.next = read_u16_le(desc1_buf + 14);

	if (req.type == kVirtioBlkTypeFlush) {
		status_desc_idx = desc0.next;
		has_data = false;
	} else {
		if ((desc1.flags & kVringDescFlagNext) == 0) {
			return;
		}
		data_desc_idx = desc0.next;
		data_len = desc1.len;
		has_data = true;
		status_desc_idx = desc1.next;
	}

	uint8_t status_desc_buf[kVringDescSize] = {};
	uint64_t status_desc_addr = queue_desc + static_cast<uint64_t>(status_desc_idx) * kVringDescSize;
	if (!read_mem(status_desc_addr, status_desc_buf, sizeof(status_desc_buf))) {
		return;
	}
	VringDesc status_desc{};
	status_desc.addr = read_u64_le(status_desc_buf);
	status_desc.len = read_u32_le(status_desc_buf + 8);
	status_desc.flags = read_u16_le(status_desc_buf + 12);

	uint8_t status = kVirtioBlkStatusOk;
	uint64_t offset = req.sector * 512ULL;

	if (req.type == kVirtioBlkTypeFlush) {
		if (image_fd >= 0 && !image_read_only) {
			if (::fsync(image_fd) != 0) {
				status = kVirtioBlkStatusIoErr;
			}
		}
	} else if (has_data) {
		if (offset + data_len > image_size_bytes) {
			status = kVirtioBlkStatusIoErr;
		} else if (req.type == kVirtioBlkTypeIn) {
			std::vector<uint8_t> buf(4096);
			uint64_t remaining = data_len;
			uint64_t guest_addr = desc1.addr;
			uint64_t file_off = offset;

			while (remaining > 0) {
				size_t chunk = static_cast<size_t>(std::min<uint64_t>(buf.size(), remaining));
				ssize_t r = ::pread(image_fd, buf.data(), chunk, static_cast<off_t>(file_off));
				if (r < 0 || static_cast<size_t>(r) != chunk) {
					status = kVirtioBlkStatusIoErr;
					break;
				}
				if (!write_mem(guest_addr, buf.data(), chunk)) {
					status = kVirtioBlkStatusIoErr;
					break;
				}
				remaining -= chunk;
				guest_addr += chunk;
				file_off += chunk;
			}
		} else if (req.type == kVirtioBlkTypeOut) {
			std::vector<uint8_t> buf(4096);
			uint64_t remaining = data_len;
			uint64_t guest_addr = desc1.addr;
			uint64_t file_off = offset;

			while (remaining > 0) {
				size_t chunk = static_cast<size_t>(std::min<uint64_t>(buf.size(), remaining));
				if (!read_mem(guest_addr, buf.data(), chunk)) {
					status = kVirtioBlkStatusIoErr;
					break;
				}
				ssize_t w = ::pwrite(image_fd, buf.data(), chunk, static_cast<off_t>(file_off));
				if (w < 0 || static_cast<size_t>(w) != chunk) {
					status = kVirtioBlkStatusIoErr;
					break;
				}
				remaining -= chunk;
				guest_addr += chunk;
				file_off += chunk;
			}
		} else {
			status = kVirtioBlkStatusUnsupp;
		}
	} else {
		status = kVirtioBlkStatusUnsupp;
	}

	if (status_desc.len >= 1) {
		write_mem(status_desc.addr, &status, 1);
	}

	uint8_t used_elem[8] = {};
	write_u32_le(used_elem, head_idx);
	write_u32_le(used_elem + 4, status == kVirtioBlkStatusOk ? data_len : 0);
	uint64_t used_elem_addr = queue_used + 4 + static_cast<uint64_t>(used_idx % queue_num) * 8;
	write_mem(used_elem_addr, used_elem, sizeof(used_elem));

	used_idx++;
	uint8_t used_idx_buf[2] = {};
	write_u16_le(used_idx_buf, used_idx);
	write_mem(queue_used + 2, used_idx_buf, sizeof(used_idx_buf));
}

void VirtioMmioBlk::reset_device() {
	host_features = kVirtioFeatureVersion1 | kVirtioBlkFeatureBlkSize;
	if (image_read_only) {
		host_features |= kVirtioBlkFeatureRo;
	}
	guest_features = 0;
	host_features_sel = 0;
	guest_features_sel = 0;
	device_status = 0;
	interrupt_status = 0;
	queue_sel = 0;
	queue_num = 0;
	queue_align = 0;
	guest_page_size = 0;
	queue_ready = 0;
	queue_desc = 0;
	queue_avail = 0;
	queue_used = 0;
	last_avail_idx = 0;
	used_idx = 0;
}

void VirtioMmioBlk::update_capacity_from_file() {
	image_size_bytes = 0;
	if (image_fd < 0) {
		return;
	}
	off_t end = ::lseek(image_fd, 0, SEEK_END);
	if (end > 0) {
		image_size_bytes = static_cast<uint64_t>(end);
	}
}

void VirtioMmioBlk::trigger_interrupt() {
	if (plic == nullptr) {
		return;
	}
	plic->gateway_trigger_interrupt(irq_number);
}

uint16_t VirtioMmioBlk::read_u16_le(const uint8_t *data) {
	return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t VirtioMmioBlk::read_u32_le(const uint8_t *data) {
	return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
	       (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

uint64_t VirtioMmioBlk::read_u64_le(const uint8_t *data) {
	uint64_t lo = read_u32_le(data);
	uint64_t hi = read_u32_le(data + 4);
	return lo | (hi << 32);
}

void VirtioMmioBlk::write_u16_le(uint8_t *data, uint16_t value) {
	data[0] = static_cast<uint8_t>(value & 0xff);
	data[1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

void VirtioMmioBlk::write_u32_le(uint8_t *data, uint32_t value) {
	data[0] = static_cast<uint8_t>(value & 0xff);
	data[1] = static_cast<uint8_t>((value >> 8) & 0xff);
	data[2] = static_cast<uint8_t>((value >> 16) & 0xff);
	data[3] = static_cast<uint8_t>((value >> 24) & 0xff);
}

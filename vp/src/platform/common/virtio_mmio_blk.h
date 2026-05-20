#pragma once

#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include <cstdint>
#include <string>
#include <systemc>
#include <vector>

#include "core/common/irq_if.h"
#include "util/initiator_if.h"
#include "util/tlm_ext_initiator.h"

class VirtioMmioBlk : public sc_core::sc_module, public initiator_if {
   public:
	tlm_utils::simple_target_socket<VirtioMmioBlk> tsock;
	tlm_utils::simple_initiator_socket<VirtioMmioBlk> isock;

	interrupt_gateway *plic = nullptr;
	uint32_t irq_number = 0;

	VirtioMmioBlk(sc_core::sc_module_name name, uint32_t irq_number);
	void set_debug(bool enable);

	bool insert_image(const std::string &path, bool read_only = false);
	void eject_image();

	void halt() override;
	std::string name() override;

   private:
	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);

	uint32_t read_reg(uint64_t offset) const;
	void write_reg(uint64_t offset, uint32_t value);

	bool read_config(uint64_t offset, uint8_t *data, size_t len) const;
	bool write_config(uint64_t offset, const uint8_t *data, size_t len);

	bool read_mem(uint64_t addr, uint8_t *data, size_t len);
	bool write_mem(uint64_t addr, const uint8_t *data, size_t len);

	void handle_queue_notify(uint32_t queue_id);
	void process_available();
	void process_descriptor_chain(uint16_t head_idx);

	void reset_device();
	void update_capacity_from_file();
	void trigger_interrupt();

	static uint16_t read_u16_le(const uint8_t *data);
	static uint32_t read_u32_le(const uint8_t *data);
	static uint64_t read_u64_le(const uint8_t *data);
	static void write_u16_le(uint8_t *data, uint16_t value);
	static void write_u32_le(uint8_t *data, uint32_t value);

   private:
	int image_fd = -1;
	bool image_read_only = false;
	uint64_t image_size_bytes = 0;
	std::string image_path;

	uint64_t host_features = 0;
	uint64_t guest_features = 0;
	uint32_t host_features_sel = 0;
	uint32_t guest_features_sel = 0;
	uint32_t device_status = 0;
	uint32_t interrupt_status = 0;
	uint32_t queue_sel = 0;
	uint16_t queue_num = 0;
	uint16_t queue_num_max = 128;
	bool debug = false;
	uint32_t queue_align = 0;
	uint32_t guest_page_size = 0;
	uint32_t queue_ready = 0;
	uint64_t queue_desc = 0;
	uint64_t queue_avail = 0;
	uint64_t queue_used = 0;
	uint16_t last_avail_idx = 0;
	uint16_t used_idx = 0;

	tlm::tlm_generic_payload trans;
	tlm_ext_initiator *ext = nullptr;
};

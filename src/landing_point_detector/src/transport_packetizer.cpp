#include "rm_image_compression/codec.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rm_image_compression
{

namespace
{

void write_u16_be(const std::uint16_t value, std::uint8_t * output)
{
  output[0] = static_cast<std::uint8_t>((value >> 8) & 0xffU);
  output[1] = static_cast<std::uint8_t>(value & 0xffU);
}

std::uint16_t read_u16_be(const std::uint8_t * input)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(input[0]) << 8) |
    static_cast<std::uint16_t>(input[1]));
}

}  // namespace

TransportPacketPacker::TransportPacketPacker(
  const std::size_t transport_packet_bytes,
  const std::size_t max_inner_packet_bytes)
: transport_packet_bytes_(transport_packet_bytes),
  max_inner_packet_bytes_(max_inner_packet_bytes)
{
  if (transport_packet_bytes_ < (kTransportLengthPrefixBytes + 1U)) {
    throw std::invalid_argument("transport_packet_bytes must leave room for at least one payload byte");
  }
  if (max_inner_packet_bytes_ == 0U) {
    throw std::invalid_argument("max_inner_packet_bytes must be positive");
  }
  if (max_inner_packet_bytes_ > kMaxLengthPrefixedPacketBytes) {
    throw std::invalid_argument("max_inner_packet_bytes must fit in a 16-bit length prefix");
  }
  if (max_inner_packet_bytes_ > (transport_packet_bytes_ - kTransportLengthPrefixBytes)) {
    throw std::invalid_argument("max_inner_packet_bytes exceeds transport capacity");
  }
}

void TransportPacketPacker::push_serialized_packet(std::vector<std::uint8_t> serialized_packet)
{
  if (serialized_packet.empty()) {
    throw std::invalid_argument("push_serialized_packet() received an empty packet");
  }
  if (serialized_packet.size() > max_inner_packet_bytes_) {
    throw std::invalid_argument("push_serialized_packet() received a packet larger than max_inner_packet_bytes");
  }

  pending_serialized_bytes_ += serialized_packet.size();
  pending_packets_.push_back(std::move(serialized_packet));
}

bool TransportPacketPacker::has_pending_packets() const noexcept
{
  return !pending_packets_.empty();
}

std::size_t TransportPacketPacker::pending_packet_count() const noexcept
{
  return pending_packets_.size();
}

std::size_t TransportPacketPacker::pending_serialized_bytes() const noexcept
{
  return pending_serialized_bytes_;
}

std::vector<std::uint8_t> TransportPacketPacker::pop_transport_packet()
{
  if (pending_packets_.empty()) {
    return {};
  }

  std::vector<std::uint8_t> output(transport_packet_bytes_, 0U);
  std::size_t write_offset = 0U;
  std::size_t pop_count = 0U;
  std::size_t popped_serialized_bytes = 0U;
  for (const std::vector<std::uint8_t> & packet : pending_packets_) {
    const std::size_t packet_size = packet.size();
    if (packet_size > max_inner_packet_bytes_) {
      throw std::runtime_error("pending packet exceeds max_inner_packet_bytes");
    }
    if (write_offset + kTransportLengthPrefixBytes + packet_size > transport_packet_bytes_) {
      break;
    }

    write_u16_be(
      static_cast<std::uint16_t>(packet_size),
      output.data() + static_cast<std::ptrdiff_t>(write_offset));
    write_offset += kTransportLengthPrefixBytes;
    std::copy(
      packet.begin(),
      packet.end(),
      output.begin() + static_cast<std::ptrdiff_t>(write_offset));
    write_offset += packet_size;
    ++pop_count;
    popped_serialized_bytes += packet_size;
  }

  if (pop_count == 0U) {
    throw std::runtime_error("no pending packet fits into one transport packet");
  }

  pending_packets_.erase(
    pending_packets_.begin(),
    pending_packets_.begin() + static_cast<std::ptrdiff_t>(pop_count));
  pending_serialized_bytes_ -= popped_serialized_bytes;
  return output;
}

std::vector<std::vector<std::uint8_t>> unpack_transport_packet(
  const std::vector<std::uint8_t> & transport_packet,
  const std::size_t max_inner_packet_bytes)
{
  if (transport_packet.empty()) {
    throw std::invalid_argument("unpack_transport_packet() received an empty transport packet");
  }
  if (max_inner_packet_bytes == 0U) {
    throw std::invalid_argument("max_inner_packet_bytes must be positive");
  }
  if (max_inner_packet_bytes > kMaxLengthPrefixedPacketBytes) {
    throw std::invalid_argument("max_inner_packet_bytes must fit in a 16-bit length prefix");
  }

  std::vector<std::vector<std::uint8_t>> packets;
  std::size_t read_offset = 0U;
  while (read_offset + kTransportLengthPrefixBytes <= transport_packet.size()) {
    const std::uint16_t packet_size = read_u16_be(
      transport_packet.data() + static_cast<std::ptrdiff_t>(read_offset));
    read_offset += kTransportLengthPrefixBytes;

    if (packet_size == 0U) {
      const bool has_non_zero_padding = std::any_of(
        transport_packet.begin() + static_cast<std::ptrdiff_t>(read_offset),
        transport_packet.end(),
        [](const std::uint8_t value) { return value != 0U; });
      if (has_non_zero_padding) {
        throw std::invalid_argument("unpack_transport_packet() found non-zero bytes after padding marker");
      }
      break;
    }

    if (packet_size > max_inner_packet_bytes) {
      throw std::invalid_argument("unpack_transport_packet() found an inner packet larger than max_inner_packet_bytes");
    }
    if (read_offset + packet_size > transport_packet.size()) {
      throw std::invalid_argument("unpack_transport_packet() found a truncated inner packet");
    }

    packets.emplace_back(
      transport_packet.begin() + static_cast<std::ptrdiff_t>(read_offset),
      transport_packet.begin() + static_cast<std::ptrdiff_t>(read_offset + packet_size));
    read_offset += packet_size;
  }

  return packets;
}

}  // namespace rm_image_compression

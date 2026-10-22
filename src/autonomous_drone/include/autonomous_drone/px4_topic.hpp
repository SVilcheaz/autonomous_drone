#pragma once

#include <string>
#include <type_traits>

namespace autonomous_drone
{
// Unversioned messages have no MESSAGE_VERSION; version zero has no suffix.
template<typename Message, typename = void>
struct MessageVersion {static constexpr unsigned value = 0;};

template<typename Message>
struct MessageVersion<Message, std::void_t<decltype(Message::MESSAGE_VERSION)>>
{static constexpr unsigned value = Message::MESSAGE_VERSION;};

template<typename Message>
std::string px4_topic(const std::string & prefix, const std::string & name)
{
  constexpr auto version = MessageVersion<Message>::value;
  return prefix + name + (version == 0 ? "" : "_v" + std::to_string(version));
}
}  // namespace autonomous_drone

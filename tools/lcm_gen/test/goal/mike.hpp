#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "papa/lima.hpp"

namespace papa {

class mike {
 public:
  std::array<double, 3> delta;
  std::array<std::array<float, 5>, 4> foxtrot;
  papa::lima alpha;
  std::string sierra;
  int32_t rows;
  int32_t cols;
  std::vector<uint8_t> bravo;
  std::vector<std::vector<int8_t>> india8;
  std::array<std::vector<int16_t>, 7> india16;
  std::vector<std::array<int32_t, 11>> india32;
  std::array<papa::lima, 2> xray;
  std::vector<papa::lima> yankee;
  std::vector<std::array<papa::lima, 2>> zulu;

  // These functions match the expected API from the legacy lcm-gen tool,
  // but note that we use `int64_t` instead of `int` for byte counts.
  //@{
  static const char* getTypeName() { return "mike"; }
  int64_t getEncodedSize() const {
    return sizeof(int64_t) + _getEncodedSizeNoHash();
  }
  int64_t _getEncodedSizeNoHash() const {
    auto [_required_size, _error] = _get_encoded_size<false>();
    return _error ? 0 : _required_size;
  }
  template <bool with_hash = true>
  int64_t encode(void* buf, int64_t offset, int64_t maxlen) const {
    auto [_required_size, _error] = _get_encoded_size<with_hash>();
    if (_error || (maxlen - offset) < _required_size) {
      return -1;
    }
    uint8_t* const _start = static_cast<uint8_t*>(buf) + offset;
    uint8_t* _cursor = _start;
    this->_encode<with_hash>(&_cursor);
    return _cursor - _start;
  }
  int64_t _encodeNoHash(void* buf, int64_t offset, int64_t maxlen) const {
    return encode<false>(buf, offset, maxlen);
  }
  template <bool with_hash = true>
  int64_t decode(const void* buf, int64_t offset, int64_t maxlen) {
    const uint8_t* const _begin = static_cast<const uint8_t*>(buf);
    const uint8_t* const _start = _begin + offset;
    const uint8_t* const _end = _begin + maxlen;
    const uint8_t* _cursor = _start;
    return this->_decode<with_hash>(&_cursor, _end) ? (_cursor - _start) : -1;
  }
  int64_t _decodeNoHash(const void* buf, int64_t offset, int64_t maxlen) {
    return decode<false>(buf, offset, maxlen);
  }
  static constexpr int64_t getHash() {
    return static_cast<int64_t>(_get_hash_impl());
  }
  template <typename Parents>
  static uint64_t _computeHash(const Parents*) {
    return getHash();
  }
  //@}

  // New-style (constexpr) hashing.
  template <size_t N = 0>
  static constexpr uint64_t _get_hash_impl(
      const std::array<uint64_t, N>& parents = {}) {
    const uint64_t base_hash = 0xd2dc16c61113f6b3ull;
    std::array<uint64_t, N + 1> new_parents{base_hash};
    for (size_t n = 0; n < N; ++n) {
      if (parents[n] == base_hash) {
        // Special case for recursive message definition.
        return 0;
      }
      new_parents[n + 1] = parents[n];
    }
    const uint64_t composite_hash = base_hash
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents);
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style size calculation with invariant checks.
  template <bool with_hash>
  std::pair<int64_t, bool /* error */> _get_encoded_size() const {
    int64_t _result = with_hash ? sizeof(int64_t) : 0;
    bool _error = false;
    int64_t _temp_result;
    bool _temp_error;
    // delta
    _result += 8 * 3;
    // foxtrot
    _result += 4 * 4 * 5;
    // alpha
    std::tie(_temp_result, _temp_error) =
        alpha._get_encoded_size<false>();
    _result += _temp_result;
    if (_temp_error) [[unlikely]] { _error = true; }
    // sierra
    _result += sizeof(int32_t) + sierra.size() + 1;
    // rows
    _result += 4;
    if (rows < 0) [[unlikely]] { _error = true; }
    // cols
    _result += 4;
    if (cols < 0) [[unlikely]] { _error = true; }
    // bravo
    _result += 1 * rows;
    if (ssize(bravo) != rows) [[unlikely]] { _error = true; }
    // india8
    _result += 1 * rows * cols;
    if (ssize(india8) != rows) [[unlikely]] { _error = true; }
    for (const auto& _india8_0 : india8) {
      if (ssize(_india8_0) != cols) [[unlikely]] { _error = true; }
    }
    // india16
    _result += 2 * 7 * cols;
    for (const auto& _india16_0 : india16) {
      if (ssize(_india16_0) != cols) [[unlikely]] { _error = true; }
    }
    // india32
    _result += 4 * rows * 11;
    if (ssize(india32) != rows) [[unlikely]] { _error = true; }
    // xray
    for (const auto& _xray_0 : xray) {
      std::tie(_temp_result, _temp_error) =
        _xray_0._get_encoded_size<false>();
      _result += _temp_result;
      if (_temp_error) [[unlikely]] { _error = true; }
    }
    // yankee
    for (const auto& _yankee_0 : yankee) {
      std::tie(_temp_result, _temp_error) =
        _yankee_0._get_encoded_size<false>();
      _result += _temp_result;
      if (_temp_error) [[unlikely]] { _error = true; }
    }
    // zulu
    for (const auto& _zulu_0 : zulu) {
      for (const auto& _zulu_1 : _zulu_0) {
        std::tie(_temp_result, _temp_error) =
        _zulu_1._get_encoded_size<false>();
        _result += _temp_result;
        if (_temp_error) [[unlikely]] { _error = true; }
      }
    }
    return {_result, _error};
  }

  // New-style encoding.
  // @pre The cursor has at least _get_encoded_size() bytes available.
  template <bool with_hash = true>
  void _encode(uint8_t** _cursor) const {
    constexpr int64_t _hash = _get_hash_impl();
    if (with_hash) { _encode_field<int64_t, 0>(_hash, _cursor); }
    _encode_field<double, 1>(delta, _cursor);
    _encode_field<float, 2>(foxtrot, _cursor);
    _encode_field<papa::lima, 0>(alpha, _cursor);
    _encode_field<std::string, 0>(sierra, _cursor);
    _encode_field<int32_t, 0>(rows, _cursor);
    _encode_field<int32_t, 0>(cols, _cursor);
    _encode_field<uint8_t, 1>(bravo, _cursor);
    _encode_field<int8_t, 2>(india8, _cursor);
    _encode_field<int16_t, 2>(india16, _cursor);
    _encode_field<int32_t, 2>(india32, _cursor);
    _encode_field<papa::lima, 1>(xray, _cursor);
    _encode_field<papa::lima, 1>(yankee, _cursor);
    _encode_field<papa::lima, 2>(zulu, _cursor);
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = _get_hash_impl();
    constexpr int64_t _hash_size = with_hash ? sizeof(_expected_hash) : 0;
    int64_t _hash = _expected_hash;
    return  // true iff success
        (*_cursor + _hash_size + 8 * 3 + 4 * 4 * 5 <= _end) &&
        (with_hash ? _decode_field(&_hash, _cursor, _end) : true) &&
        (_hash == _expected_hash) &&
        _decode_field(&delta, _cursor, _end, ArrayDims<1>{3}) &&
        _decode_field(&foxtrot, _cursor, _end, ArrayDims<2>{4, 5}) &&
        _decode_field(&alpha, _cursor, _end) &&
        _decode_field(&sierra, _cursor, _end) &&
        (*_cursor + 4 + 4 <= _end) &&
        _decode_field(&rows, _cursor, _end) &&
        (rows >= 0) &&
        _decode_field(&cols, _cursor, _end) &&
        (cols >= 0) &&
        (*_cursor + 1 * rows + 1 * rows * cols + 2 * 7 * cols + 4 * rows * 11 <= _end) &&
        _decode_field(&bravo, _cursor, _end, ArrayDims<1>{rows}) &&
        _decode_field(&india8, _cursor, _end, ArrayDims<2>{rows, cols}) &&
        _decode_field(&india16, _cursor, _end, ArrayDims<2>{7, cols}) &&
        _decode_field(&india32, _cursor, _end, ArrayDims<2>{rows, 11}) &&
        _decode_field(&xray, _cursor, _end, ArrayDims<1>{2}) &&
        _decode_field(&yankee, _cursor, _end, ArrayDims<1>{rows}) &&
        _decode_field(&zulu, _cursor, _end, ArrayDims<2>{rows, 2});
  }

 private:
  // TODO(jwnimmer-tri) Remove once we have C++20.
  template <typename Container>
  static int64_t ssize(const Container& c) {
    return static_cast<int64_t>(c.size());
  }

  // Given an N-byte integer at `_input` in network byte order, returns it as
  // a host unsigned integer using the matching unsigned integer type. (This
  // is also used to convert host to network order; it's the same operation.)
  template <size_t N>
  static auto _byteswap(const void* _input) {
    // clang-format off
    using result_t = std::conditional_t<
        N == 1, uint8_t, std::conditional_t<
        N == 2, uint16_t, std::conditional_t<
        N == 4, uint32_t, std::conditional_t<
        N == 8, uint64_t, void>>>>;
    // clang-format on
    result_t _result;
    std::memcpy(&_result, _input, N);
    // TODO(jwnimmer-tri) Don't bswap on PowerPC.
    if constexpr (N == 1) {
      return _result;
    } else if constexpr (N == 2) {
      return __builtin_bswap16(_result);
    } else if constexpr (N == 4) {
      return __builtin_bswap32(_result);
    } else if constexpr (N == 8) {
      return __builtin_bswap64(_result);
    }
  }

  template <size_t N>
  static void _memcpy_bswap(uint8_t** _cursor, const void* _src,
                            size_t _bytes) {
    if constexpr (N == 1) {
      if (_bytes > 0) {
        std::memcpy(*_cursor, _src, _bytes);
        *_cursor += _bytes;
      }
    } else {
      const uint8_t* const _src_bytes = reinterpret_cast<const uint8_t*>(_src);
      for (size_t _i = 0; _i < _bytes; _i += N) {
        auto _swapped = _byteswap<N>(_src_bytes + _i);
        std::memcpy(*_cursor, &_swapped, N);
        *_cursor += N;
      }
    }
  }

  // The dimensions of an array, for use during encoding / decoding, e.g., for
  // a message field `int8_t image[6][4]` we'd use `ArrayDims<2>{6, 4}`.
  template <size_t ndims>
  using ArrayDims = std::array<int64_t, ndims>;

  // Returns the second and following elements of _dims (i.e., _dims[1:]).
  // https://en.wikipedia.org/wiki/CAR_and_CDR
  template <size_t ndims>
  static ArrayDims<ndims - 1> _cdr(const std::array<int64_t, ndims>& _dims) {
    static_assert(ndims > 0);
    ArrayDims<ndims - 1> _result;
    for (size_t i = 1; i < ndims; ++i) {
      _result[i - 1] = _dims[i];
    }
    return _result;
  }

  // Returns true iff T has a "slab" layout in memory, where all of its data
  // lives in one block of contiguous memory. The template arguments are the
  // same as _encode_field().
  template <typename Base, int ndims, typename T>
  static constexpr bool _is_slab() {
    if constexpr (std::is_fundamental_v<Base> && ndims > 0) {
      return std::is_trivial_v<typename T::value_type>;
    }
    return false;
  }

  // Given a field (or child element within a field), encodes it into the given
  // byte cursor and advances the cursor. As with _encode(), we assume as a
  // precondition that the cursor has sufficient writeable space.
  template <typename Base, int ndims, typename T>
  static void _encode_field(const T& _input, uint8_t** _cursor) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (std::is_fundamental_v<Base> && ndims == 0) {
      // Encode a single POD value.
      static_assert(std::is_same_v<Base, T>);
      _memcpy_bswap<sizeof(T)>(_cursor, &_input, sizeof(T));
    } else if constexpr (_is_slab<Base, ndims, T>()) {
      // Encode a slab of POD memory.
      const size_t _raw_size = _input.size() * sizeof(_input[0]);
      _memcpy_bswap<sizeof(Base)>(_cursor, _input.data(), _raw_size);
    } else if constexpr (ndims > 0) {
      // Encode a non-slab array.
      for (const auto& _child : _input) {
        _encode_field<Base, ndims - 1>(_child, _cursor);
      }
    } else if constexpr (std::is_same_v<T, std::string>) {
      // Encode a string.
      const int32_t _size = _input.size() + 1;
      _encode_field<int32_t, 0>(_size, _cursor);
      std::memcpy(*_cursor, _input.c_str(), _size);
      *_cursor += _size;
    } else {
      // Encode a struct.
      _input.template _encode<false>(_cursor);
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success. The array `_dims` and storage order follow the same pattern as in
  // _encode_field(); refer to those docs for details. When T is a POD type or
  // an LCM array with a POD type as its base, checking that the cursor has
  // enough readable bytes is a pre-condition that must met by the calling code
  // inside _decode().
  template <typename T, size_t ndims = 0>
  static bool _decode_field(T* _output, const uint8_t** _cursor,
                            const uint8_t* _end,
                            const ArrayDims<ndims>& _dims = {}) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (ndims == 0) {
      // With no array dimensions, just decode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD output.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(*_cursor);
        std::memcpy(_output, &_swapped, N);
        *_cursor += N;
        // Overflow checking is the responsibility of the top-level _decode.
        (void)(_end);
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String output.
        int32_t _size{};
        const bool ok = (*_cursor + sizeof(_size) <= _end) &&
                        _decode_field(&_size, _cursor, _end) &&
                        (_size > 0) && (*_cursor + _size <= _end);
        if (ok) {
          _output->replace(_output->begin(), _output->end(), *_cursor,
                           *_cursor + _size - 1);
        }
        *_cursor += _size;
        return ok;
      } else {
        // Struct output.
        return _output->template _decode<false>(_cursor, _end);
      }
    } else {
      // In case of a variable-size dimension, resize our storage to match.
      if constexpr (std::is_same_v<T, std::vector<typename T::value_type>>) {
        _output->resize(_dims[0]);
      }
      // Decode each sub-item in turn.
      for (auto& _child : *_output) {
        if (!_decode_field(&_child, _cursor, _end, _cdr(_dims))) {
          return false;
        }
      }
      return true;
    }
  }
};

}  // namespace papa

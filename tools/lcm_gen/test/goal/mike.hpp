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
    uint8_t* const _begin = static_cast<uint8_t*>(buf);
    uint8_t* const _start = _begin + offset;
    uint8_t* const _end = _start + maxlen;
    auto [_required_size, _error] = _get_encoded_size<with_hash>();
    if (_error || (_end - _start) < _required_size) [[unlikely]] {
      return -1;
    }
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
    const uint8_t* const _end = _start + maxlen;
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
    _result += sizeof(int32_t) + ssize(sierra) + 1;
    if (ssize(sierra) >= INT32_MAX) [[unlikely]] { _error = true; }
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
    if constexpr (with_hash) {
      constexpr int64_t _hash = _get_hash_impl();
      _encode_field(_hash, _cursor);
    }
    _encode_field(delta, _cursor, ArrayDims<1>{3});
    _encode_field(foxtrot, _cursor, ArrayDims<2>{4, 5});
    _encode_field(alpha, _cursor);
    _encode_field(sierra, _cursor);
    _encode_field(rows, _cursor);
    _encode_field(cols, _cursor);
    _encode_field(bravo, _cursor, ArrayDims<1>{rows});
    _encode_field(india8, _cursor, ArrayDims<2>{rows, cols});
    _encode_field(india16, _cursor, ArrayDims<2>{7, cols});
    _encode_field(india32, _cursor, ArrayDims<2>{rows, 11});
    _encode_field(xray, _cursor, ArrayDims<1>{2});
    _encode_field(yankee, _cursor, ArrayDims<1>{rows});
    _encode_field(zulu, _cursor, ArrayDims<2>{rows, 2});
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = _get_hash_impl();
    int64_t _hash = _expected_hash;
    return  // true iff success
        (with_hash ? _decode_field(&_hash, _cursor, _end) : true) &&
        (_hash == _expected_hash) &&
        _decode_field(&delta, _cursor, _end, ArrayDims<1>{3}) &&
        _decode_field(&foxtrot, _cursor, _end, ArrayDims<2>{4, 5}) &&
        _decode_field(&alpha, _cursor, _end) &&
        _decode_field(&sierra, _cursor, _end) &&
        _decode_field(&rows, _cursor, _end) &&
        (rows >= 0) &&
        _decode_field(&cols, _cursor, _end) &&
        (cols >= 0) &&
        _decode_field(&bravo, _cursor, _end, ArrayDims<1>{rows}) &&
        _decode_field(&india8, _cursor, _end, ArrayDims<2>{rows, cols}) &&
        _decode_field(&india16, _cursor, _end, ArrayDims<2>{7, cols}) &&
        _decode_field(&india32, _cursor, _end, ArrayDims<2>{rows, 11}) &&
        _decode_field(&xray, _cursor, _end, ArrayDims<1>{2}) &&
        _decode_field(&yankee, _cursor, _end, ArrayDims<1>{rows}) &&
        _decode_field(&zulu, _cursor, _end, ArrayDims<2>{rows, 2});
  }

 private:
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

  // Returns true iff T has a "slab" layout in memory, where all of its data
  // lives in one block of contiguous memory. The template arguments are the
  // same as _encode_field().
  template <typename T, int ndims>
  static constexpr bool _is_slab() {
    if constexpr (ndims == 0) {
      return false;
    } else {
      using Element = typename T::value_type;
      if constexpr (!std::is_trivial_v<Element>) {
        return false;
      } else if constexpr (ndims == 1) {
        return std::is_fundamental_v<Element>;
      } else {
        return _is_slab<Element, ndims - 1>();
      }
    }
  }

  // Returns the size of the ndims'th element type of the given containter T.
  // (This tells us how big of a byteswap we'll need while copying T's slab.)
  template <typename T, int ndims>
  static constexpr size_t _get_slab_step() {
    if constexpr (ndims > 0) {
      return _get_slab_step<typename T::value_type, ndims - 1>();
    }
    return sizeof(T);
  }

  // Copies _bytes amount of data from _src to _dst. While copying, each group
  // of N bytes is byteswapped. The number of _bytes must be a multiple of N.
  template <size_t N>
  static void _memcpy_byteswap(void* _dst, const void* _src, size_t _bytes) {
    if constexpr (N == 1) {
      if (_bytes > 0) [[likely]] {
        std::memcpy(_dst, _src, _bytes);
      }
    } else {
      for (size_t _i = 0; _i < _bytes; _i += N) {
        auto _swapped = _byteswap<N>(_src);
        std::memcpy(_dst, &_swapped, N);
        _dst = static_cast<uint8_t*>(_dst) + N;
        _src = static_cast<const uint8_t*>(_src) + N;
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

  // Given a field (or child element within a field), encodes it into the given
  // byte cursor and advances the cursor. As with _encode(), we assume as a
  // precondition that the cursor has sufficient writeable space. Arrays are
  // passed with `_input` as vector-like container and `_dims` as the list of
  // multi-dimensional vector sizes, e.g., `int8_t image[6][4]` would be called
  // like `_encode_field(image.at(0), &cursor, ArrayDims<2>{6, 4})`. In LCM
  // messages, multi-dimensional arrays are encoded using C's memory layout
  // (i.e., with the last dimension as the most tightly packed.)
  template <typename T, size_t ndims = 0>
  static void _encode_field(const T& _input, uint8_t** _cursor,
                            const ArrayDims<ndims>& _dims = ArrayDims<0>{}) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (ndims == 0) {
      // With no array dimensions, just encode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD input.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(&_input);
        std::memcpy(*_cursor, &_swapped, N);
        *_cursor += N;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String input.
        const int32_t _size = ssize(_input) + 1;
        _encode_field(_size, _cursor);
        std::memcpy(*_cursor, _input.c_str(), _size);
        *_cursor += _size;
      } else {
        // Struct input.
        _input.template _encode<false>(_cursor);
      }
    } else {
      if constexpr (_is_slab<T, ndims>()){
        // Encode a slab of POD memory.
        const size_t _raw_size = _input.size() * sizeof(_input[0]);
        constexpr size_t N = _get_slab_step<T, ndims>();
        _memcpy_byteswap<N>(*_cursor, _input.data(), _raw_size);
        *_cursor += _raw_size;
      } else {
        // Encode each sub-item in turn, forwarding all _dims but the first.
        for (const auto& _child : _input) {
          _encode_field(_child, _cursor, _cdr(_dims));
        }
      }
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success. The array `_dims` and storage order follow the same pattern as in
  // _encode_field(); refer to those docs for details.
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
        if (*_cursor + N > _end) {
          return false;
        }
        auto _swapped = _byteswap<N>(*_cursor);
        std::memcpy(_output, &_swapped, N);
        *_cursor += N;
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String output.
        int32_t _size{};
        const bool ok = _decode_field(&_size, _cursor, _end) &&
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
      if constexpr (_is_slab<T, ndims>()) {
        // Decode a slab of POD memory.
        const size_t _raw_size = _dims[0] * sizeof((*_output)[0]);
        if ((*_cursor + _raw_size) > _end) {
          return false;
        }
        constexpr size_t N = _get_slab_step<T, ndims>();
        _memcpy_byteswap<N>(_output->data(), *_cursor, _raw_size);
        *_cursor += _raw_size;
      } else {
        // Decode each sub-item in turn.
        for (auto& _child : *_output) {
          if (!_decode_field(&_child, _cursor, _end, _cdr(_dims))) {
            return false;
          }
        }
      }
      return true;
    }
  }
};

}  // namespace papa

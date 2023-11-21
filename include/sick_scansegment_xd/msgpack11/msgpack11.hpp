#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <initializer_list>
#include <istream>
#include <ostream>
#include <sstream>


#ifdef _MSC_VER
    #if _MSC_VER <= 1800 // VS 2013
        #ifndef noexcept
            #define noexcept throw()
        #endif

        #ifndef snprintf
            #define snprintf _snprintf_s
        #endif
    #endif
#endif

namespace msgpack11 {

class MsgPackValue;

class MsgPack final {
public:
    // Types
    // NB: all ints are numbers but not all numbers are ints.
    enum Type {
        NUMBER      = 1,
        INT         = 2 | NUMBER,
        NUL         = 1 << 2,
        FLOAT32     = 2 << 2 | NUMBER,
        FLOAT64     = 3 << 2 | NUMBER,
        INT8        = 4 << 2 | INT,
        INT16       = 5 << 2 | INT,
        INT32       = 6 << 2 | INT,
        INT64       = 7 << 2 | INT,
        UINT8       = 8 << 2 | INT,
        UINT16      = 9 << 2 | INT,
        UINT32      = 10 << 2 | INT,
        UINT64      = 11 << 2 | INT,
        BOOL        = 12 << 2,
        STRING      = 13 << 2,
        BINARY      = 14 << 2,
        ARRAY       = 15 << 2,
        OBJECT      = 16 << 2,
        EXTENSION   = 17 << 2
    };

    // Array and object typedefs
    typedef std::vector<MsgPack> array;
    typedef std::map<MsgPack, MsgPack> object;

    // Binary and extension typedefs
    typedef std::vector<uint8_t> binary;
    typedef std::tuple<int8_t, binary> extension;

    // Constructors for the various types of JSON value.
    MsgPack() noexcept;                // NUL
    MsgPack(std::nullptr_t) noexcept;  // NUL
    MsgPack(float value);              // FLOAT32
    MsgPack(double value);             // FLOAT64
    MsgPack(int8_t value);             // INT8
    MsgPack(int16_t value);            // INT16
    MsgPack(int32_t value);            // INT32
    MsgPack(int64_t value);            // INT64
    MsgPack(uint8_t value);            // UINT8
    MsgPack(uint16_t value);           // UINT16
    MsgPack(uint32_t value);           // UINT32
    MsgPack(uint64_t value);           // UINT64
    MsgPack(bool value);               // BOOL
    MsgPack(const std::string &value); // STRING
    MsgPack(std::string &&value);      // STRING
    MsgPack(const char * value);       // STRING
    MsgPack(const array &values);      // ARRAY
    MsgPack(array &&values);           // ARRAY
    MsgPack(const object &values);     // OBJECT
    MsgPack(object &&values);          // OBJECT
    MsgPack(const binary &values);     // BINARY
    MsgPack(binary &&values);          // BINARY
    MsgPack(const extension &values);  // EXTENSION
    MsgPack(extension &&values);       // EXTENSION

    // Implicit constructor: anything with a to_msgpack() function.
    template <class T, class = decltype(&T::to_msgpack)>
    MsgPack(const T & t) : MsgPack(t.to_msgpack()) {}

    // Implicit constructor: map-like objects (std::map, std::unordered_map, etc)
    template <class M, typename std::enable_if<
        std::is_constructible<MsgPack, typename M::key_type>::value
        && std::is_constructible<MsgPack, typename M::mapped_type>::value,
            int>::type = 0>
    MsgPack(const M & m) : MsgPack(object(m.begin(), m.end())) {}

    // Implicit constructor: vector-like objects (std::list, std::vector, std::set, etc)
    template <class V, typename std::enable_if<
        std::is_constructible<MsgPack, typename V::value_type>::value &&
        !std::is_same<typename binary::value_type, typename V::value_type>::value,
            int>::type = 0>
    MsgPack(const V & v) : MsgPack(array(v.begin(), v.end())) {}

    template <class V, typename std::enable_if<
        std::is_constructible<MsgPack, typename V::value_type>::value &&
        std::is_same<typename binary::value_type, typename V::value_type>::value,
            int>::type = 0>
    MsgPack(const V & v) : MsgPack(binary(v.begin(), v.end())) {}

    // This prevents MsgPack(some_pointer) from accidentally producing a bool. Use
    // MsgPack(bool(some_pointer)) if that behavior is desired.
    MsgPack(void *) = delete;

    // Accessors
    Type type() const;

    bool is_null()      const { return type() == NUL; }
    bool is_bool()      const { return type() == BOOL; }
    bool is_number()    const { return type() & NUMBER; }
    bool is_float32()   const { return type() == FLOAT32; }
    bool is_float64()   const { return type() == FLOAT64; }
    bool is_int()       const { return (type() & INT) == INT; }
    bool is_int8()      const { return type() == INT8; }
    bool is_int16()     const { return type() == INT16; }
    bool is_int32()     const { return type() == INT32; }
    bool is_int64()     const { return type() == INT64; }
    bool is_uint8()     const { return type() == UINT8; }
    bool is_uint16()    const { return type() == UINT16; }
    bool is_uint32()    const { return type() == UINT32; }
    bool is_uint64()    const { return type() == UINT64; }
    bool is_string()    const { return type() == STRING; }
    bool is_array()     const { return type() == ARRAY; }
    bool is_binary()    const { return type() == BINARY; }
    bool is_object()    const { return type() == OBJECT; }
    bool is_extension() const { return type() == EXTENSION; }

    // Return the enclosed value if this is a number, 0 otherwise. Note that msgpack11 does not
    // distinguish between integer and non-integer numbers - number_value() and int_value()
    // can both be applied to a NUMBER-typed object.
    double number_value() const;
    float float32_value() const;
    double float64_value() const;
    int32_t int_value() const;
    int8_t int8_value() const;
    int16_t int16_value() const;
    int32_t int32_value() const;
    int64_t int64_value() const;
    uint8_t uint8_value() const;
    uint16_t uint16_value() const;
    uint32_t uint32_value() const;
    uint64_t uint64_value() const;

    // Return the enclosed value if this is a boolean, false otherwise.
    bool bool_value() const;
    // Return the enclosed string if this is a string, "" otherwise.
    const std::string &string_value() const;
    // Return the enclosed std::vector if this is an array, or an empty vector otherwise.
    const array &array_items() const;
    // Return the enclosed std::map if this is an object, or an empty map otherwise.
    const object &object_items() const;
    // Return the enclosed std::vector if this is an binary, or an empty map otherwise.
    const binary &binary_items() const;
    // Return the enclosed std::tuple if this is an extension, or an empty map otherwise.
    const extension &extension_items() const;

    // Return a reference to arr[i] if this is an array, MsgPack() otherwise.
    const MsgPack & operator[](size_t i) const;
    // Return a reference to obj[key] if this is an object, MsgPack() otherwise.
    const MsgPack & operator[](const std::string &key) const;

    // Serialize.
    void dump(std::string &out) const {
        std::stringstream ss;
        ss << *this;
        out = ss.str();
    }
    
    std::string dump() const {
        std::stringstream ss;
        ss << *this;
        return ss.str();
    }
    
    friend std::ostream& operator<<(std::ostream& os, const MsgPack& msgpack);
    // Parse. If parse fails, set msgpack to MsgPack() and
    // sets failbit on stream.
    friend std::istream& operator>>(std::istream& is, MsgPack& msgpack);

    // Parse. If parse fails, return MsgPack() and assign
    // an error message to err.
    static MsgPack parse(const std::string & in, std::string & err);
    // Parse. If parse fails, return MsgPack(), sets failbit on stream and and
    // assign an error message to err.
    static MsgPack parse(std::istream& is, std::string &err);
    // Parse (without the need to default initialise object first).
    // If parse fails, return MsgPack() and sets failbit on stream.
    static MsgPack parse(std::istream& is);
    static MsgPack parse(const char * in, size_t len, std::string & err) {
        if (in) {
            return parse(std::string(in,in+len), err);
        } else {
            err = "null input";
            return nullptr;
        }
    }
    // Parse multiple objects, concatenated or separated by whitespace
    static std::vector<MsgPack> parse_multi(
        const std::string & in,
        std::string::size_type & parser_stop_pos,
        std::string & err);

    static inline std::vector<MsgPack> parse_multi(
        const std::string & in,
        std::string & err) {
        std::string::size_type parser_stop_pos;
        return parse_multi(in, parser_stop_pos, err);
    }

    bool operator== (const MsgPack &rhs) const;
    bool operator<  (const MsgPack &rhs) const;
    bool operator!= (const MsgPack &rhs) const { return !(*this == rhs); }
    bool operator<= (const MsgPack &rhs) const { return !(rhs < *this); }
    bool operator>  (const MsgPack &rhs) const { return  (rhs < *this); }
    bool operator>= (const MsgPack &rhs) const { return !(*this < rhs); }

    /* has_shape(types, err)
     *
     * Return true if this is a JSON object and, for each item in types, has a field of
     * the given type. If not, return false and set err to a descriptive message.
     */
    typedef std::initializer_list<std::pair<std::string, Type>> shape;
    bool has_shape(const shape & types, std::string & err) const;

private:
    std::shared_ptr<MsgPackValue> m_ptr;
};

} // namespace msgpack11

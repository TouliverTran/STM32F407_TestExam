#ifndef __MISC_H
#define __MISC_H



typedef signed char		    s8;
typedef unsigned char		u8;
typedef short			    s16;
typedef unsigned short		u16;
typedef int			        s32;
typedef unsigned		    u32;
typedef long long		    s64;
typedef unsigned long long	u64;

typedef signed char		    __s8;
typedef unsigned char		__u8;
typedef short			    __s16;
typedef unsigned short		__u16;
typedef int			        __s32;
typedef unsigned		    __u32;
typedef long long		    __s64;
typedef unsigned long long	__u64;

#define BITS_TO_LONGS(nr) (((nr) + (8 * sizeof(long) - 1)) / (8 * sizeof(long)))

/**
 * __struct_group() - Create a mirrored named and anonyomous struct
 *
 * @TAG: The tag name for the named sub-struct (usually empty)
 * @NAME: The identifier name of the mirrored sub-struct
 * @ATTRS: Any struct attributes (usually empty)
 * @MEMBERS: The member declarations for the mirrored structs
 *
 * Used to create an anonymous union of two structs with identical layout
 * and size: one anonymous and one named. The former's members can be used
 * normally without sub-struct naming, and the latter can be used to
 * reason about the start, end, and size of the group of struct members.
 * The named struct can also be explicitly tagged for layer reuse, as well
 * as both having struct attributes appended.
 */
#define __struct_group(TAG, NAME, ATTRS, MEMBERS...) \
	union { \
		struct { MEMBERS } ATTRS; \
		struct TAG { MEMBERS } ATTRS NAME; \
	}
	
#define DECLARE_BITMAP(name,bits) \
	unsigned long name[BITS_TO_LONGS(bits)]

/**
 * struct_group() - Wrap a set of declarations in a mirrored struct
 *
 * @NAME: The identifier name of the mirrored sub-struct
 * @MEMBERS: The member declarations for the mirrored structs
 *
 * Used to create an anonymous union of two structs with identical
 * layout and size: one anonymous and one named. The former can be
 * used normally without sub-struct naming, and the latter can be
 * used to reason about the start, end, and size of the group of
 * struct members.
 */
#define struct_group(NAME, MEMBERS...)	\
	__struct_group(/* no tag */, NAME, /* no attrs */, MEMBERS)

/*
 * The 64-bit value is not atomic on 32-bit systems - you MUST NOT read it
 * without sampling the sequence number in jiffies_lock.
 * get_jiffies_64() will do this for you as appropriate.
 *
 * jiffies and jiffies_64 are at the same address for little-endian systems
 * and for 64-bit big-endian systems.
 * On 32-bit big-endian systems, jiffies is the lower 32 bits of jiffies_64
 * (i.e., at address @jiffies_64 + 4).
 * See arch/ARCH/kernel/vmlinux.lds.S
 */
extern unsigned long volatile jiffies;

#ifdef CONFIG_CPU_BIG_ENDIAN
#define __BIG_ENDIAN 4321
#else
#define __LITTLE_ENDIAN 1234
#endif

#define __ARG_PLACEHOLDER_1 0,
#define __take_second_arg(__ignored, val, ...) val

/*
 * The use of "&&" / "||" is limited in certain expressions.
 * The following enable to calculate "and" / "or" with macro expansion only.
 */
#define __and(x, y)			___and(x, y)
#define ___and(x, y)			____and(__ARG_PLACEHOLDER_##x, y)
#define ____and(arg1_or_junk, y)	__take_second_arg(arg1_or_junk y, 0)

#define __or(x, y)			___or(x, y)
#define ___or(x, y)			____or(__ARG_PLACEHOLDER_##x, y)
#define ____or(arg1_or_junk, y)		__take_second_arg(arg1_or_junk 1, y)

/*
 * Helper macros to use CONFIG_ options in C/CPP expressions. Note that
 * these only work with boolean and tristate options.
 */

/*
 * Getting something that works in C and CPP for an arg that may or may
 * not be defined is tricky.  Here, if we have "#define CONFIG_BOOGER 1"
 * we match on the placeholder define, insert the "0," for arg1 and generate
 * the triplet (0, 1, 0).  Then the last step cherry picks the 2nd arg (a one).
 * When CONFIG_BOOGER is not defined, we generate a (... 1, 0) pair, and when
 * the last step cherry picks the 2nd arg, we get a zero.
 */
#define __is_defined(x)			___is_defined(x)
#define ___is_defined(val)		____is_defined(__ARG_PLACEHOLDER_##val)
#define ____is_defined(arg1_or_junk)	__take_second_arg(arg1_or_junk 1, 0)

/*
 * IS_BUILTIN(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'y', 0
 * otherwise. For boolean options, this is equivalent to
 * IS_ENABLED(CONFIG_FOO).
 */
#define IS_BUILTIN(option) __is_defined(option)

/*
 * IS_MODULE(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'm', 0
 * otherwise.  CONFIG_FOO=m results in "#define CONFIG_FOO_MODULE 1" in
 * autoconf.h.
 */
#define IS_MODULE(option) __is_defined(option##_MODULE)

/*
 * IS_REACHABLE(CONFIG_FOO) evaluates to 1 if the currently compiled
 * code can call a function defined in code compiled based on CONFIG_FOO.
 * This is similar to IS_ENABLED(), but returns false when invoked from
 * built-in code when CONFIG_FOO is set to 'm'.
 */
#define IS_REACHABLE(option) __or(IS_BUILTIN(option), \
				__and(IS_MODULE(option), __is_defined(MODULE)))

/*
 * IS_ENABLED(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'y' or 'm',
 * 0 otherwise.  Note that CONFIG_FOO=y results in "#define CONFIG_FOO 1" in
 * autoconf.h, while CONFIG_FOO=m results in "#define CONFIG_FOO_MODULE 1".
 */
#define IS_ENABLED(option) __or(IS_BUILTIN(option), IS_MODULE(option))

#define true 0x1u
#define false 0x0u

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
/**
 * struct mii_bus - Represents an MDIO bus
 *
 * @owner: Who owns this device
 * @name: User friendly name for this MDIO device, or driver name
 * @id: Unique identifier for this bus, typical from bus hierarchy
 * @priv: Driver private data
 *
 * The Bus class for PHYs.  Devices which provide access to
 * PHYs should register using this structure
 */
struct mii_bus {
	u8 mdio_lock;
};

struct device {
	const char		*init_name; /* initial name of the device */
};

struct mdio_device {
	struct device dev;
	struct mii_bus *bus;
};

static inline void mutex_lock(u8 *mtx)
{
	return false;
}

static inline void mutex_unlock(u8 *mtx)
{
	return false;
}

static inline void mutex_is_locked(u8 *mtx)
{
	return false;
}

#endif /* __MISC_H */
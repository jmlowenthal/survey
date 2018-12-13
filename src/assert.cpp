#if defined(BOOST_ENABLE_ASSERT_HANDLER) || ( defined(BOOST_ENABLE_ASSERT_DEBUG_HANDLER) && !defined(NDEBUG) )

#include <stdexcept>
#include <cstdio>
#include <boost/assert.hpp>
#include <boost/format.hpp>

void boost::assertion_failed(
    char const * expr,
    char const * function,
    char const * file,
    long line
) {
    throw std::runtime_error(
        boost::str(
            boost::format("%s:%d %s: %s") % file % line % function % expr
        )
    );
}

void boost::assertion_failed_msg(
    char const * expr,
    char const * msg,
    char const * function,
    char const * file,
    long line
) {
    throw std::runtime_error(
        boost::str(
            boost::format("%s:%d %s: %s %s")
                % file % line % function % expr % msg
        )
    );
}

#endif
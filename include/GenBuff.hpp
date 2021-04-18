#ifndef GEN_BUF_HPP
#define GEN_BUF_HPP

#include <cstddef> /* std::size_t */
#include <memory>  /* std::unique_ptr */
#include <utility> /* std::pair */

using gen_buf_t = std::pair<std::unique_ptr<char[]>, std::size_t>;

#endif /* GEN_BUF_HPP */

#pragma once
#include "perception/logging.hpp"
namespace perception::detail {
template <class F> decltype(auto) boundary(ErrorCode code, const char *component, F &&f) {
    try {
        return f();
    } catch (const std::exception &e) {
        throw PerceptionException(errorFromException(code, e, component));
    } catch (...) {
        throw PerceptionException(code, "non-standard exception", component);
    }
}
inline void log(const std::shared_ptr<Logger> &logger, LogLevel level, const char *event,
                const char *component, const std::string &text = {}) noexcept {
    if (logger)
        logger->write(level, event, component, text);
}
template <class F>
decltype(auto) initialization(const std::shared_ptr<Logger> &logger, ErrorCode code,
                              const char *component, F &&f) {
    try {
        return f();
    } catch (const std::exception &e) {
        auto error = withErrorContext(errorFromException(code, e, component), code);
        if (logger)
            logger->error(error);
        throw PerceptionException(std::move(error));
    } catch (...) {
        PerceptionException error(code, "non-standard initialization exception", component);
        if (logger)
            logger->error(error.info());
        throw error;
    }
}
} // namespace perception::detail

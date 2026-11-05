#include "jdxl/jdxl.h"

const char *jdxl_result_str(jdxl_result_t result)
{
        switch (result) {
        case JDXL_OK:                  return "ok";
        case JDXL_ERR_NULL_ARG:        return "null argument";
        case JDXL_ERR_INVALID_ID:      return "invalid id";
        case JDXL_ERR_INVALID_ARG:     return "invalid argument";
        case JDXL_ERR_EMPTY:           return "empty";
        case JDXL_ERR_DUPLICATE_ID:    return "duplicate id";
        case JDXL_ERR_TOO_MANY_STATUS: return "too many status replies";
        case JDXL_ERR_TOO_LONG:        return "too long";
        default:                       return "unknown";
        }
}
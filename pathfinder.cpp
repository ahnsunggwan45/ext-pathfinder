#include "php_pathfinder.h"

extern "C" {
#include "ext/standard/info.h"
#include "Zend/zend_exceptions.h"
#include "Zend/zend_interfaces.h"
#include "Zend/zend_API.h"
}

#include "src/AStarSolver.h"
#include "src/NavMesh.h"
#include "src/PathCache.h"

#include <cstring>
#include <new>
#include <utility>
#include <vector>

using namespace pathfinder;

// ============================================================================
// Object glue
// ============================================================================

struct NavMeshObject {
    NavMesh              nav;
    AStarSolver          solver;
    PathCache            cache;
    std::vector<int32_t> pathBuffer;
    bool                 cacheEnabled;
    zend_object          std; // MUST be last — Zend reaches forward via XtOffsetOf
};

static zend_class_entry    *navmesh_ce = nullptr;
static zend_object_handlers navmesh_handlers;

static inline NavMeshObject *navmesh_from_obj(zend_object *obj) {
    return reinterpret_cast<NavMeshObject *>(
        reinterpret_cast<char *>(obj) - XtOffsetOf(NavMeshObject, std));
}

static inline NavMeshObject *navmesh_from_zval(zval *z) {
    return navmesh_from_obj(Z_OBJ_P(z));
}

static zend_object *navmesh_create(zend_class_entry *ce) {
    NavMeshObject *intern = static_cast<NavMeshObject *>(
        zend_object_alloc(sizeof(NavMeshObject), ce));

    new (&intern->nav)        NavMesh();
    new (&intern->solver)     AStarSolver();
    new (&intern->cache)      PathCache(1024);
    new (&intern->pathBuffer) std::vector<int32_t>();
    intern->cacheEnabled = true;

    zend_object_std_init(&intern->std, ce);
    object_properties_init(&intern->std, ce);
    intern->std.handlers = &navmesh_handlers;

    return &intern->std;
}

static void navmesh_free(zend_object *obj) {
    NavMeshObject *intern = navmesh_from_obj(obj);
    intern->pathBuffer.~vector();
    intern->cache.~PathCache();
    intern->solver.~AStarSolver();
    intern->nav.~NavMesh();
    zend_object_std_dtor(obj);
}

// ============================================================================
// arginfo
// ============================================================================

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_void, 0, 0, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_setBlockProperty, 0, 0, 3)
    ZEND_ARG_TYPE_INFO(0, blockStateId, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, passable,     _IS_BOOL, 0)
    ZEND_ARG_TYPE_INFO(0, solid,        _IS_BOOL, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_setBlockProperties, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, properties, IS_ARRAY, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_loadSubChunk, 0, 0, 4)
    ZEND_ARG_TYPE_INFO(0, cx,             IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, cy,             IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, cz,             IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, packedBlockIds, IS_STRING, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_loadSubChunkFromWordArray, 0, 0, 6)
    ZEND_ARG_TYPE_INFO(0, cx,            IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, cy,            IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, cz,            IS_LONG,   0)
    ZEND_ARG_TYPE_INFO(0, wordArray,     IS_STRING, 0)
    ZEND_ARG_TYPE_INFO(0, palette,       IS_ARRAY,  0)
    ZEND_ARG_TYPE_INFO(0, bitsPerBlock,  IS_LONG,   0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_subChunkCoord, 0, 0, 3)
    ZEND_ARG_TYPE_INFO(0, cx, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, cy, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, cz, IS_LONG, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_unloadColumn, 0, 0, 2)
    ZEND_ARG_TYPE_INFO(0, cx, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, cz, IS_LONG, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_updateBlock, 0, 0, 4)
    ZEND_ARG_TYPE_INFO(0, x,            IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, y,            IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, z,            IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, blockStateId, IS_LONG, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_findPath, 0, 0, 6)
    ZEND_ARG_TYPE_INFO(0, sx, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, sy, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, sz, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, ex, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, ey, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, ez, IS_LONG, 0)
    ZEND_ARG_INFO(0, options)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(arginfo_navmesh_setCacheSize, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, maxEntries, IS_LONG, 0)
ZEND_END_ARG_INFO()

// ============================================================================
// Methods
// ============================================================================

PHP_METHOD(NavMesh, __construct) {
    ZEND_PARSE_PARAMETERS_NONE();
    // All initialisation is done in navmesh_create.
}

PHP_METHOD(NavMesh, setBlockProperty) {
    zend_long blockId;
    bool      passable, solid;

    ZEND_PARSE_PARAMETERS_START(3, 3)
        Z_PARAM_LONG(blockId)
        Z_PARAM_BOOL(passable)
        Z_PARAM_BOOL(solid)
    ZEND_PARSE_PARAMETERS_END();

    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);
    intern->nav.blockTable().set(static_cast<uint32_t>(blockId), passable, solid);
}

PHP_METHOD(NavMesh, setBlockProperties) {
    HashTable *props;
    ZEND_PARSE_PARAMETERS_START(1, 1)
        Z_PARAM_ARRAY_HT(props)
    ZEND_PARSE_PARAMETERS_END();

    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);
    BlockTable    &table  = intern->nav.blockTable();

    zend_string *strKey;
    zend_ulong   numKey;
    zval        *value;
    ZEND_HASH_FOREACH_KEY_VAL(props, numKey, strKey, value) {
        if (strKey != nullptr) {
            zend_throw_error(nullptr, "setBlockProperties: keys must be integer block-state IDs");
            RETURN_THROWS();
        }
        if (Z_TYPE_P(value) != IS_ARRAY) {
            zend_throw_error(nullptr, "setBlockProperties: each value must be a [passable, solid] array");
            RETURN_THROWS();
        }
        HashTable *pair      = Z_ARRVAL_P(value);
        zval      *zPassable = zend_hash_index_find(pair, 0);
        zval      *zSolid    = zend_hash_index_find(pair, 1);
        if (zPassable == nullptr || zSolid == nullptr) {
            zend_throw_error(nullptr, "setBlockProperties: pair must have indices [0]=passable, [1]=solid");
            RETURN_THROWS();
        }
        table.set(static_cast<uint32_t>(numKey), zend_is_true(zPassable), zend_is_true(zSolid));
    } ZEND_HASH_FOREACH_END();
}

PHP_METHOD(NavMesh, clearBlockTable) {
    ZEND_PARSE_PARAMETERS_NONE();
    navmesh_from_zval(ZEND_THIS)->nav.blockTable().clear();
}

PHP_METHOD(NavMesh, loadSubChunk) {
    zend_long    cx, cy, cz;
    zend_string *packed;

    ZEND_PARSE_PARAMETERS_START(4, 4)
        Z_PARAM_LONG(cx)
        Z_PARAM_LONG(cy)
        Z_PARAM_LONG(cz)
        Z_PARAM_STR(packed)
    ZEND_PARSE_PARAMETERS_END();

    constexpr size_t expected = SubChunk::VOLUME * sizeof(uint32_t);
    if (ZSTR_LEN(packed) != expected) {
        zend_throw_error(nullptr,
            "loadSubChunk: expected %zu bytes (%d × uint32), got %zu",
            expected, SubChunk::VOLUME, ZSTR_LEN(packed));
        RETURN_THROWS();
    }

    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);
    intern->nav.loadSubChunk(
        static_cast<int32_t>(cx),
        static_cast<int32_t>(cy),
        static_cast<int32_t>(cz),
        reinterpret_cast<const uint32_t *>(ZSTR_VAL(packed)));
}

PHP_METHOD(NavMesh, loadSubChunkFromWordArray) {
    zend_long    cx, cy, cz, bitsPerBlock;
    zend_string *wordArray;
    HashTable   *paletteHt;

    ZEND_PARSE_PARAMETERS_START(6, 6)
        Z_PARAM_LONG(cx)
        Z_PARAM_LONG(cy)
        Z_PARAM_LONG(cz)
        Z_PARAM_STR(wordArray)
        Z_PARAM_ARRAY_HT(paletteHt)
        Z_PARAM_LONG(bitsPerBlock)
    ZEND_PARSE_PARAMETERS_END();

    // Flatten palette HashTable → vector<uint32_t> for O(1) C++ lookup.
    // chunkutils2 returns getPalette() as a 0-indexed list, so we honour insertion order.
    const size_t paletteSize = zend_hash_num_elements(paletteHt);
    if (paletteSize == 0) {
        zend_throw_error(nullptr, "loadSubChunkFromWordArray: palette must not be empty");
        RETURN_THROWS();
    }
    if (paletteSize > 65536) {
        zend_throw_error(nullptr, "loadSubChunkFromWordArray: palette too large (%zu entries)", paletteSize);
        RETURN_THROWS();
    }

    std::vector<uint32_t> palette;
    palette.reserve(paletteSize);
    zval *entry;
    ZEND_HASH_FOREACH_VAL(paletteHt, entry) {
        palette.push_back(static_cast<uint32_t>(zval_get_long(entry)));
    } ZEND_HASH_FOREACH_END();

    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);

    const char *err = nullptr;
    const bool  ok  = intern->nav.loadSubChunkFromWordArray(
        static_cast<int32_t>(cx),
        static_cast<int32_t>(cy),
        static_cast<int32_t>(cz),
        reinterpret_cast<const uint8_t *>(ZSTR_VAL(wordArray)),
        ZSTR_LEN(wordArray),
        palette.data(),
        palette.size(),
        static_cast<int>(bitsPerBlock),
        &err
    );
    if (!ok) {
        zend_throw_error(nullptr, "loadSubChunkFromWordArray: %s", err ? err : "unknown error");
        RETURN_THROWS();
    }
}

PHP_METHOD(NavMesh, loadAirSubChunk) {
    zend_long cx, cy, cz;
    ZEND_PARSE_PARAMETERS_START(3, 3)
        Z_PARAM_LONG(cx) Z_PARAM_LONG(cy) Z_PARAM_LONG(cz)
    ZEND_PARSE_PARAMETERS_END();
    navmesh_from_zval(ZEND_THIS)->nav.loadAirSubChunk(
        static_cast<int32_t>(cx), static_cast<int32_t>(cy), static_cast<int32_t>(cz));
}

PHP_METHOD(NavMesh, loadSolidSubChunk) {
    zend_long cx, cy, cz;
    ZEND_PARSE_PARAMETERS_START(3, 3)
        Z_PARAM_LONG(cx) Z_PARAM_LONG(cy) Z_PARAM_LONG(cz)
    ZEND_PARSE_PARAMETERS_END();
    navmesh_from_zval(ZEND_THIS)->nav.loadSolidSubChunk(
        static_cast<int32_t>(cx), static_cast<int32_t>(cy), static_cast<int32_t>(cz));
}

PHP_METHOD(NavMesh, unloadSubChunk) {
    zend_long cx, cy, cz;
    ZEND_PARSE_PARAMETERS_START(3, 3)
        Z_PARAM_LONG(cx) Z_PARAM_LONG(cy) Z_PARAM_LONG(cz)
    ZEND_PARSE_PARAMETERS_END();
    navmesh_from_zval(ZEND_THIS)->nav.unloadSubChunk(
        static_cast<int32_t>(cx), static_cast<int32_t>(cy), static_cast<int32_t>(cz));
}

PHP_METHOD(NavMesh, unloadColumn) {
    zend_long cx, cz;
    ZEND_PARSE_PARAMETERS_START(2, 2)
        Z_PARAM_LONG(cx) Z_PARAM_LONG(cz)
    ZEND_PARSE_PARAMETERS_END();
    navmesh_from_zval(ZEND_THIS)->nav.unloadColumn(
        static_cast<int32_t>(cx), static_cast<int32_t>(cz));
}

PHP_METHOD(NavMesh, isLoaded) {
    zend_long cx, cy, cz;
    ZEND_PARSE_PARAMETERS_START(3, 3)
        Z_PARAM_LONG(cx) Z_PARAM_LONG(cy) Z_PARAM_LONG(cz)
    ZEND_PARSE_PARAMETERS_END();
    RETURN_BOOL(navmesh_from_zval(ZEND_THIS)->nav.isLoaded(
        static_cast<int32_t>(cx), static_cast<int32_t>(cy), static_cast<int32_t>(cz)));
}

PHP_METHOD(NavMesh, clear) {
    ZEND_PARSE_PARAMETERS_NONE();
    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);
    intern->nav.clear();
    intern->cache.clear();
}

PHP_METHOD(NavMesh, updateBlock) {
    zend_long x, y, z, blockId;
    ZEND_PARSE_PARAMETERS_START(4, 4)
        Z_PARAM_LONG(x) Z_PARAM_LONG(y) Z_PARAM_LONG(z) Z_PARAM_LONG(blockId)
    ZEND_PARSE_PARAMETERS_END();
    navmesh_from_zval(ZEND_THIS)->nav.updateBlock(
        static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z),
        static_cast<uint32_t>(blockId));
}

// ----------------------------------------------------------------------------
// findPath
// ----------------------------------------------------------------------------

static inline void emit_path_array(zval *return_value, const std::vector<int32_t> &path) {
    array_init(return_value);
    const size_t triplets = path.size() / 3;
    for (size_t i = 0; i < triplets; ++i) {
        zval entry;
        array_init_size(&entry, 3);
        add_next_index_long(&entry, path[i * 3 + 0]);
        add_next_index_long(&entry, path[i * 3 + 1]);
        add_next_index_long(&entry, path[i * 3 + 2]);
        add_next_index_zval(return_value, &entry);
    }
}

PHP_METHOD(NavMesh, findPath) {
    zend_long  sx, sy, sz, ex, ey, ez;
    HashTable *opts = nullptr;

    ZEND_PARSE_PARAMETERS_START(6, 7)
        Z_PARAM_LONG(sx) Z_PARAM_LONG(sy) Z_PARAM_LONG(sz)
        Z_PARAM_LONG(ex) Z_PARAM_LONG(ey) Z_PARAM_LONG(ez)
        Z_PARAM_OPTIONAL
        Z_PARAM_ARRAY_HT_OR_NULL(opts)
    ZEND_PARSE_PARAMETERS_END();

    AStarConfig cfg;
    bool        useCache = true;

    if (opts != nullptr) {
        zval *v;
        #define READ_LONG(field, name)                                                              \
            if ((v = zend_hash_str_find(opts, name, sizeof(name) - 1)) != nullptr) {                \
                cfg.field = static_cast<int32_t>(zval_get_long(v));                                  \
            }
        #define READ_FLOAT(field, name)                                                             \
            if ((v = zend_hash_str_find(opts, name, sizeof(name) - 1)) != nullptr) {                \
                cfg.field = static_cast<float>(zval_get_double(v));                                  \
            }
        #define READ_BOOL(field, name)                                                              \
            if ((v = zend_hash_str_find(opts, name, sizeof(name) - 1)) != nullptr) {                \
                cfg.field = zend_is_true(v);                                                         \
            }

        READ_LONG (maxIterations,   "maxIterations")
        READ_LONG (maxStepUp,       "maxStepUp")
        READ_LONG (maxFallDistance, "maxFallDistance")
        READ_BOOL (allowDiagonal,   "allowDiagonal")
        READ_LONG (entityWidth,     "entityWidth")
        READ_LONG (entityHeight,    "entityHeight")
        READ_FLOAT(diagonalCost,    "diagonalCost")
        READ_FLOAT(cardinalCost,    "cardinalCost")
        READ_FLOAT(stepUpCost,      "stepUpCost")
        READ_FLOAT(fallCost,        "fallCost")
        READ_LONG (maxPathLength,   "maxPathLength")

        if ((v = zend_hash_str_find(opts, "useCache", sizeof("useCache") - 1)) != nullptr) {
            useCache = zend_is_true(v);
        }

        #undef READ_LONG
        #undef READ_FLOAT
        #undef READ_BOOL
    }

    NavMeshObject *intern     = navmesh_from_zval(ZEND_THIS);
    const uint64_t currentGen = intern->nav.generation();

    PathKey key{
        static_cast<int32_t>(sx), static_cast<int32_t>(sy), static_cast<int32_t>(sz),
        static_cast<int32_t>(ex), static_cast<int32_t>(ey), static_cast<int32_t>(ez),
        cfg.entityWidth,           cfg.entityHeight,
    };

    // ----- Cache lookup ---------------------------------------------------------------------
    if (useCache && intern->cacheEnabled) {
        PathCache::Entry cached;
        if (intern->cache.get(key, currentGen, cached)) {
            if (!cached.found) RETURN_NULL();
            emit_path_array(return_value, cached.path);
            return;
        }
    }

    // ----- Solve ----------------------------------------------------------------------------
    intern->pathBuffer.clear();
    const bool found = intern->solver.findPath(
        intern->nav,
        static_cast<int32_t>(sx), static_cast<int32_t>(sy), static_cast<int32_t>(sz),
        static_cast<int32_t>(ex), static_cast<int32_t>(ey), static_cast<int32_t>(ez),
        cfg, intern->pathBuffer);

    // ----- Cache store ----------------------------------------------------------------------
    if (useCache && intern->cacheEnabled) {
        intern->cache.put(key, intern->pathBuffer, found, currentGen);
    }

    if (!found) RETURN_NULL();
    emit_path_array(return_value, intern->pathBuffer);
}

// ----------------------------------------------------------------------------
// Inspection / cache control
// ----------------------------------------------------------------------------

PHP_METHOD(NavMesh, getGeneration) {
    ZEND_PARSE_PARAMETERS_NONE();
    RETURN_LONG(static_cast<zend_long>(navmesh_from_zval(ZEND_THIS)->nav.generation()));
}

PHP_METHOD(NavMesh, getLoadedSubChunkCount) {
    ZEND_PARSE_PARAMETERS_NONE();
    RETURN_LONG(static_cast<zend_long>(navmesh_from_zval(ZEND_THIS)->nav.loadedSubChunkCount()));
}

PHP_METHOD(NavMesh, getLastIterations) {
    ZEND_PARSE_PARAMETERS_NONE();
    RETURN_LONG(static_cast<zend_long>(navmesh_from_zval(ZEND_THIS)->solver.lastIterations()));
}

PHP_METHOD(NavMesh, setCacheSize) {
    zend_long n;
    ZEND_PARSE_PARAMETERS_START(1, 1)
        Z_PARAM_LONG(n)
    ZEND_PARSE_PARAMETERS_END();

    if (n < 0) n = 0;
    NavMeshObject *intern = navmesh_from_zval(ZEND_THIS);
    if (n == 0) {
        intern->cacheEnabled = false;
        intern->cache.clear();
    } else {
        intern->cacheEnabled = true;
        intern->cache.setMaxSize(static_cast<size_t>(n));
    }
}

PHP_METHOD(NavMesh, clearCache) {
    ZEND_PARSE_PARAMETERS_NONE();
    navmesh_from_zval(ZEND_THIS)->cache.clear();
}

PHP_METHOD(NavMesh, getCacheSize) {
    ZEND_PARSE_PARAMETERS_NONE();
    RETURN_LONG(static_cast<zend_long>(navmesh_from_zval(ZEND_THIS)->cache.size()));
}

// ============================================================================
// Method table
// ============================================================================

static const zend_function_entry navmesh_methods[] = {
    PHP_ME(NavMesh, __construct,            arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, setBlockProperty,       arginfo_navmesh_setBlockProperty,   ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, setBlockProperties,     arginfo_navmesh_setBlockProperties, ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, clearBlockTable,        arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, loadSubChunk,             arginfo_navmesh_loadSubChunk,             ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, loadSubChunkFromWordArray, arginfo_navmesh_loadSubChunkFromWordArray, ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, loadAirSubChunk,        arginfo_navmesh_subChunkCoord,      ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, loadSolidSubChunk,      arginfo_navmesh_subChunkCoord,      ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, unloadSubChunk,         arginfo_navmesh_subChunkCoord,      ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, unloadColumn,           arginfo_navmesh_unloadColumn,       ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, isLoaded,               arginfo_navmesh_subChunkCoord,      ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, clear,                  arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, updateBlock,            arginfo_navmesh_updateBlock,        ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, findPath,               arginfo_navmesh_findPath,           ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, getGeneration,          arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, getLoadedSubChunkCount, arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, getLastIterations,      arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, setCacheSize,           arginfo_navmesh_setCacheSize,       ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, clearCache,             arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_ME(NavMesh, getCacheSize,           arginfo_navmesh_void,               ZEND_ACC_PUBLIC)
    PHP_FE_END
};

// ============================================================================
// Module init
// ============================================================================

PHP_MINIT_FUNCTION(pathfinder) {
    zend_class_entry ce;
    INIT_NS_CLASS_ENTRY(ce, "pathfinder", "NavMesh", navmesh_methods);
    navmesh_ce               = zend_register_internal_class(&ce);
    navmesh_ce->create_object = navmesh_create;
    navmesh_ce->ce_flags |= ZEND_ACC_FINAL;

    memcpy(&navmesh_handlers, zend_get_std_object_handlers(), sizeof(zend_object_handlers));
    navmesh_handlers.offset    = XtOffsetOf(NavMeshObject, std);
    navmesh_handlers.free_obj  = navmesh_free;
    navmesh_handlers.clone_obj = nullptr; // NavMesh is non-cloneable

    return SUCCESS;
}

PHP_MINFO_FUNCTION(pathfinder) {
    php_info_print_table_start();
    php_info_print_table_header(2, "pathfinder support", "enabled");
    php_info_print_table_row(2,   "version",            PHP_PATHFINDER_VERSION);
    php_info_print_table_end();
}

zend_module_entry pathfinder_module_entry = {
    STANDARD_MODULE_HEADER,
    PHP_PATHFINDER_EXTNAME,
    nullptr,                 // functions
    PHP_MINIT(pathfinder),
    nullptr,                 // MSHUTDOWN
    nullptr,                 // RINIT
    nullptr,                 // RSHUTDOWN
    PHP_MINFO(pathfinder),
    PHP_PATHFINDER_VERSION,
    STANDARD_MODULE_PROPERTIES
};

#ifdef COMPILE_DL_PATHFINDER
# ifdef ZTS
ZEND_TSRMLS_CACHE_DEFINE()
# endif
ZEND_GET_MODULE(pathfinder)
#endif

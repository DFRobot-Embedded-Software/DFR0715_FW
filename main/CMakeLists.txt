set(srcs
    main.c
    src/hardDevInit.c
    )

set(requires
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS inc
                       REQUIRES ${requires})

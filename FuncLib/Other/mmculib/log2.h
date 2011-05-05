#ifndef _LOG2_H
#define _LOG2_H

/*
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<[8, 16, 32, 64].
 * Inefficient algorithm, intended for compile-time constants.
 */
#define LOG2_8BIT(v)  (8 - 90/(((v)/4+14)|1) - 2/((v)/2+1))
#define LOG2_16BIT(v) (8*((v)>255) + LOG2_8BIT((v) >>8*((v)>255)))
#define LOG2_32BIT(v) \
    (16*((v)>65535L) + LOG2_16BIT((v)*1L >>16*((v)>65535L)))
#define LOG2_64BIT(v)\
    (32*((v)/2L>>31 > 0) \
     + LOG2_32BIT((v)*1L >>16*((v)/2L>>31 > 0) \
                         >>16*((v)/2L>>31 > 0)))


/*
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<512.
 * Huge computation, intended for compile-time constants.
 */
#define LOG2_512BIT(val)\
        LOG2_TMP_FINISH(val,\
        LOG2_TMP_CHUNK(0, val,\
        LOG2_TMP_CHUNK(1, val,\
        LOG2_TMP_CHUNK(3, val,\
        LOG2_TMP_CHUNK(7, val,\
        LOG2_TMP_CHUNK(15, val,\
        LOG2_TMP_CHUNK(31, val, 0)))))))
#define LOG2_TMP_FINISH(val, known_bits)\
        (LOG2_8BIT((val) >> known_bits) + known_bits)
#define LOG2_TMP_CHUNK(ones, val, known_bits)\
        (((val)*1L >>known_bits \
          >>8>>ones>>ones>>ones>>ones>>ones>>ones>>ones>>ones \
          > 0) * 8*(ones+1) + known_bits)


/*
 * Define  Name = (val ? floor(log2(val)) : 0)  as an enum constant.
 * Expects 0 <= val < 1<<512.  Tries to force a compile error otherwise.
 * Note: Uses some <Name>__<letter/digit> identifiers.
 */
#define DEFINE_LOG2(Name, val)\
enum {\
    Name##__5= LOG2_TMP_CHUNK(31, val, 0),\
    Name##__4= LOG2_TMP_CHUNK(15, val, Name##__5),\
    Name##__3= LOG2_TMP_CHUNK( 7, val, Name##__4),\
    Name##__2= LOG2_TMP_CHUNK( 3, val, Name##__3),\
    Name##__1= LOG2_TMP_CHUNK( 1, val, Name##__2),\
    Name##__0= LOG2_TMP_CHUNK( 0, val, Name##__1),\
    Name = LOG2_TMP_FINISH(val, Name##__0),\
    Name##__c= (val)>>Name == !!(val) ? 1 : -999 \
}; 


#endif

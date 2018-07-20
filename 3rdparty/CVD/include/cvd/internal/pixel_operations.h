
#ifndef CVD_PIXEL_OPERATIONS_H_
#define CVD_PIXEL_OPERATIONS_H_

#include <cmath>
#include <cvd/internal/pixel_traits.h>
#include <cvd/internal/convert_pixel_types.h>
#include <cvd/internal/builtin_components.h>
#include <cvd/internal/rgb_components.h>

namespace CVD {

  namespace Pixel {
    
    
// operations::assign<DestType>(dest,src)
template <class T, unsigned int N = Component<T>::count> struct operations {
    inline static void assign(T & lhs, const T & rhs) { memcpy(&lhs, &rhs, sizeof(T)); }
    template <class S> inline static void assign(T & lhs, const S & rhs) { for (unsigned int i=0; i<N; i++) Component<T>::get(lhs,i) = (typename Component<T>::type)Component<S>::get(rhs,i); }
    template <class S> inline static void add(T & lhs, const S & rhs) { for (unsigned int i=0;i<N;++i) Component<T>::get(lhs,i) += Component<S>::get(rhs,i); }
    template <class S> inline static void subtract(T & lhs, const S & rhs) { for (unsigned int i=0;i<N;++i) Component<T>::get(lhs,i) -= Component<S>::get(rhs,i); }
    template <class S> inline static void multiply(T & lhs, const S& rhs) { for (unsigned int i=0;i<N;++i) Component<T>::get(lhs,i) = (typename Component<T>::type)(Component<T>::get(lhs,i)*rhs); }
    template <class S> inline static void divide(T & lhs, const S& rhs) { for (unsigned int i=0;i<N;++i) Component<T>::get(lhs,i) = (typename Component<T>::type)(Component<T>::get(lhs,i)/rhs); }
    inline static bool equal(const T & a, const T & b) { return memcmp(&a,&b,sizeof(T)) == 0; }
    inline static void zero(T & t) { memset(&t, 0, sizeof(T)); }
};

template <class T> struct operations<T,1> {
    template <class S> inline static void assign(T& lhs, const S& rhs) { lhs = (T)rhs; }
    template <class S> inline static void add(T& lhs, const S& rhs) { lhs += rhs; }
    template <class S> inline static void subtract(T& lhs, const S& rhs) { lhs -= rhs; }
    template <class S> inline static void multiply(T& lhs, const S& rhs) { lhs = (T)(lhs*rhs); }
    template <class S> inline static void divide(T& lhs, const S& rhs) { lhs = (T)(lhs/rhs); }
    inline static bool equal(const T& a, const T& b) { return a == b; }
    inline static void zero(T& t) { t = T(); }
};

template <class T, unsigned int N=Pixel::Component<T>::count>
struct difference {
    typedef typename Pixel::Component<T>::type TComp;
    typedef typename Pixel::traits<TComp>::wider_type diff_type;
    static inline diff_type avgabs(const T& a, const T& b) {
        diff_type sum = diff_type();
        for (unsigned int i=0; i<N; i++)
            sum += std::abs((diff_type)Pixel::Component<T>::get(a,i) - Pixel::Component<T>::get(b,i));
        return sum/N;
    }
    static inline diff_type avg(const T& a, const T& b) {
        diff_type sum = diff_type();
        for (unsigned int i=0; i<N; i++)
            sum += (diff_type)Pixel::Component<T>::get(a,i) - Pixel::Component<T>::get(b,i);
        return sum/N;
    }
};

template <class T>
struct difference<T, 1> {
    typedef typename Pixel::Component<T>::type TComp;
    typedef typename Pixel::traits<TComp>::wider_type diff_type;
    static inline diff_type avgabs(const T& a, const T& b) {
        return CVD::abs((diff_type)Pixel::Component<T>::get(a,0) - Pixel::Component<T>::get(b,0));
    }
    static inline diff_type avg(const T& a, const T& b) {
        return (diff_type)Pixel::Component<T>::get(a,0) - (diff_type)Pixel::Component<T>::get(b,0);
    }
};

};

};

#endif

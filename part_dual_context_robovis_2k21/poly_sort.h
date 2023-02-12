
// #pragma once em caso de rodar no Windows Visual Studio // o path só é add uma vez
							  // quando é feita a compilação

#include <algorithm>  // std::iter_swap // troca os valores de dois objetos
#include <iterator>   // std::advance, std::iterator_traits
#include <functional> // std::less


template <typename iterator, typename LessThan>
void mySort_Poly(iterator primeiro, iterator ultimo, LessThan lessThan)
{
  for (iterator actual = primeiro; actual != ultimo; ++actual)
  {

//encontre o menor elemento dentre os ñ ordenados  e lembre-se de seu iterador no "mínimo"
    iterator menor_possivel = actual;
    iterator compare = actual;
    ++compare;

    // percorrer todos os elementos ainda n ordenados
    while (compare != ultimo)
    {
      // quando encontrar o novo minimo, salve
      if (lessThan(*compare, *menor_possivel))
        menor_possivel = compare;
     
      ++compare;
    }

    // adcionar o mínimo ao final
    if (actual != menor_possivel)
      std::iter_swap(actual, menor_possivel);
  }
}




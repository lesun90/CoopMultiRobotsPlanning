#include "Utils/DLHandler.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/PseudoRandom.hpp"
#include <cstring>


int main(int argc, char **argv)
{
  typedef int (*MainFcn) (int, char **);

  MP::RandomSeed();

  if(argc < 2)
  {
    BeginPrint(PRINT_WARNING);
    printf("usage: Runner <program_name> [list_of_program_args]\n");
    EndPrint(PRINT_WARNING);
  }
  else
  {
    MainFcn fcn = (MainFcn) MP::DLHandler::GetSymbol(argv[1]);
    if(fcn)
    {
      BeginPrint(PRINT_INFO);
      printf("Executing program <%s>\n", argv[1]);
      EndPrint(PRINT_INFO);
      return fcn(argc - 1, &(argv[1]));
    }
    else
    {
      BeginPrint(PRINT_ERROR);
      printf("Program <%s> not found\n", argv[1]);
      EndPrint(PRINT_ERROR);
    }
  }

  return 0;
}

/**
 * @file   sr_automatic_pid_tuning.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Sep 22 11:36:52 2011
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Automatic pid tuning using a genetic algorithm, based on the evodev
 *         library.
 *
 *
 */
#include <iostream>

#include <es/make_real.h>
#include <apply.h>

#include "sr_automatic_pid_tuning/fitness.hpp"

using namespace std;

int main(int argc, char* argv[])
{

  try
  {
  typedef eoReal<eoMinimizingFitness> EOT;

  eoParser parser(argc, argv);  // for user-parameter reading

  eoState state;    // keeps all things allocated

  ///// FIRST, problem or representation dependent stuff
  //////////////////////////////////////////////////////

  // The evaluation fn - encapsulated into an eval counter for output
  eoEvalFuncPtr<EOT, double, const std::vector<double>&>
	       mainEval( real_value );
  eoEvalFuncCounter<EOT> eval(mainEval);

  // the genotype - through a genotype initializer
  eoRealInitBounded<EOT>& init = make_genotype(parser, state, EOT());

  // Build the variation operator (any seq/prop construct)
  eoGenOp<EOT>& op = make_op(parser, state, init);

  //// Now the representation-independent things
  //////////////////////////////////////////////

  // initialize the population - and evaluate
  // yes, this is representation indepedent once you have an eoInit
  eoPop<EOT>& pop   = make_pop(parser, state, init);

  // stopping criteria
  eoContinue<EOT> & term = make_continue(parser, state, eval);
  // output
  eoCheckPoint<EOT> & checkpoint = make_checkpoint(parser, state, eval, term);
  // algorithm (need the operator!)
  eoAlgo<EOT>& ea = make_algo_scalar(parser, state, eval, checkpoint, op);

  ///// End of construction of the algorith
  /////////////////////////////////////////
  // to be called AFTER all parameters have been read!!!
  make_help(parser);

  //// GO
  ///////
  // evaluate intial population AFTER help and status in case it takes time
  apply<EOT>(eval, pop);
  // print it out
  cout << "Initial Population\n";
  pop.sortedPrintOn(cout);
  cout << endl;

  run_ea(ea, pop); // run the ea

  cout << "Final Population\n";
  pop.sortedPrintOn(cout);
  cout << endl;
  }
  catch(exception& e)
  {
    cout << e.what() << endl;
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

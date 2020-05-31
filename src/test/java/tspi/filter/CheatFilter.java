package tspi.filter;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ensemble;
import tspi.simulator.Trajectory;

/** Uses the Trajectory model to get the state, therefore all errors should be zero. */
class CheatFilter implements Filter {
    double time;
    Trajectory cheat;

    public CheatFilter(Trajectory hint) {
        //just pass it through...
        this.cheat = hint;
    }

    @Override
    public RealVector filter(double time, Ensemble ensemble) {
        this.time = time;
        //below is replaced by the filter state[?]
        return cheat.getState(time);
    }

    @Override
    public RealVector getState() {
        return cheat.getState(time);
    }

    @Override
    public RealMatrix getCovariance() { return null; }

    @Override
    public RealVector getResiduals(){
        return null;
    }

    @Override
    public RealVector getInnovations(){
        return null;
    }

}
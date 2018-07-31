package MotionProfiling.MP_Classes;

public class NumericIntegral {

	public interface Function {
		double function(double x);
	}
	
	
	public static double numericIntegral(Function func, double x0, double xEnd, double dx){
		double sum = 0;
		double lastFx = func.function(x0);
		double fx;
		for (double i = x0 + dx; i < xEnd; i += dx){
			fx = func.function(i);
			//System.out.println(fx);
			sum += ((fx + lastFx) / 2) * dx;
			lastFx = fx;
		}
		
		sum += ((func.function(xEnd) + lastFx) / 2) * dx;
		
		return sum;
	}
	
}

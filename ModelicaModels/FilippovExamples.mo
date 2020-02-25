within ;
package FilippovExamples
  package Example1
    model Stick_slip "Stick-Slip System without Filippov solution technique"
      Real x1(start=0);
      Real x2(start=0);
      Integer z1(start=1);
      Real h;
    equation
      der(x1) = x2*z1 + x2*(1-z1);
      der(x2) = (-x1+(1/(1.2-x2)))*z1+(-x1-(1/(0.8+x2)))*(1-z1);
      h = x2 - 0.2;
      when h < 0 then
        z1 = 1;
      elsewhen h > 0 then
        z1 = 0;
      end when;
      annotation (Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"1\"><tr>
<td><p>Last update</p></td>
<td><p>25/02/2020</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>M.A.A Murad and L. Vanfretti</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mohammed:murad@ucdconnect.ie\">mohammed.murad@ucdconnect.ie</a></p></td>
</tr>
</table>
</html>"));
    end Stick_slip;

    model Stick_slip_filippov "Stick-Slip System considering Filippov sliding mode"
      Real x1(start=0);
      Real x2(start=0);
      Integer z1(start=1);
      Integer z2(start=0);
      Real r1;
      Real r2;
      Real h;
      Real a;
    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing(enable = true);
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing1(enable = true);
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing2(enable = true);
    equation
      der(x1) = x2*z1*(1-z2) + x2*(1-z1)*(1-z2)+ x2*z2;
      der(x2) = (-x1+(1/(1.2-x2)))*z1*(1-z2) + (-x1-(1/(0.8+x2)))*(1-z1)*(1-z2);
      h = x2 - 0.2;
      r1 = -x1+1;
      r2 = -x1-1;
      a = (-x1+1)/2;
      zeroCrossing.u = h;
      zeroCrossing1.u = (-x1+1)/2;
      zeroCrossing2.u = 1-a;
      when zeroCrossing.y then
           if r1*r2 < 0 then
             if r1 > 0 and r2 < 0 then
             z1 = pre(z1);
             z2 = 1;
             else
             z1 = pre(z1);
             z2 = 0;
             end if;
           elseif r1*r2 > 0 then
             if r1 < 0 then
             z1 = 1;
             z2 = 0;
             elseif r1 > 0 then
             z1 = 0;
             z2 = 0;
             else
             z1 = pre(z1);
             z2 = 0;
             end if;
           else
             z1 = pre(z1);
             z2 = 0;
           end if;
     elsewhen zeroCrossing1.y and pre(z2)==1 then
             z1 = 1;
             z2 = 0;
     elsewhen zeroCrossing2.y and pre(z2)==1 then
             z1 = 0;
             z2 = 0;
      end when;
      annotation (Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"1\"><tr>
<td><p>Last update</p></td>
<td><p>25/02/2020</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>M.A.A Murad and L. Vanfretti</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mohammed:murad@ucdconnect.ie\">mohammed.murad@ucdconnect.ie</a></p></td>
</tr>
</table>
</html>"));
    end Stick_slip_filippov;

  end Example1;

  package Example2
    model RelayFeedback "A direct implementation of Relay Feedback System"
      parameter Real zt = 0.05;
      parameter Real w = 25;
      parameter Real s = -1;
      parameter Real l = 1;
      Real x1(start=0.05);
      Real x2(start=-0.01);
      Real x3(start=0.1);
      Integer z1(start=0);
      Real h;
    equation
      der(x1) = (-(2*zt*w+l)*x1+x2+1)*z1+(-(2*zt*w+l)*x1+x2-1)*(1-z1);
      der(x2) = (-(2*zt*w*l+w*w)*x1+x3+2*s)*z1+(-(2*zt*w*l+w*w)*x1+x3-2*s)*(1-z1);
      der(x3) = (-l*w*w*x1+1)*z1+(-l*w*w*x1-1)*(1-z1);
      h = x1;
      when h < 0 then
        z1 = 1;
      elsewhen h > 0 then
        z1 = 0;
      end when;
      annotation (Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"1\"><tr>
<td><p>Last update</p></td>
<td><p>25/02/2020</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>M.A.A Murad and L. Vanfretti</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mohammed:murad@ucdconnect.ie\">mohammed.murad@ucdconnect.ie</a></p></td>
</tr>
</table>
</html>"));
    end RelayFeedback;

    model RelayFeedback_filippov
      "A Relay Feedback System Using Filippov Solution Concept"
      parameter Real zt = 0.05;
      parameter Real w = 25;
      parameter Real s = -1;
      parameter Real l = 1;
      Real x1(start=0.05);
      Real x2(start=-0.01);
      Real x3(start=0.1);
      Integer z1(start=0);
      Real h;
      Real z2(start=0);
      Real r1;
      Real r2;
      Real a;
      Real b;
      Real c;
    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing(enable = true);
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing1(enable = true);
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing2(enable = true);
    equation
      der(x1) = (-(2*zt*w+l)*x1+x2+1)*z1*(1-z2)+(-(2*zt*w+l)*x1+x2-1)*(1-z1)*(1-z2)+0*z2;
      der(x2) =  b*z1*(1-z2)+(-(2*zt*w*l+w*w)*x1+x3-2*s)*(1-z1)*(1-z2)+(b-4*s*a)*z2;
      der(x3) =  c*z1*(1-z2)+(-l*w*w*x1-1)*(1-z1)*(1-z2)+(c-2*a)*z2;
      a = (-(2*zt*w+l)*x1+x2+1)/2;
      b = -(2*zt*w*l+w*w)*x1+x3+2*s;
      c = -l*w*w*x1+1;
      h = x1;
      r1 = (-(2*zt*w+l)*x1+x2+1);
      r2 = (-(2*zt*w+l)*x1+x2-1);
      zeroCrossing.u = h;
      zeroCrossing1.u = a;
      zeroCrossing2.u = 1-a;
      when zeroCrossing.y then
           if r1*r2 < 0 then
             if r1 > 0 and r2 < 0 then
             z1 = pre(z1);
             z2 = 1;
             else
             z1 = pre(z1);
             z2 = 0;
             end if;
           elseif r1*r2 > 0 then
             if r1 < 0 then
             z1 = 1;
             z2 = 0;
             elseif r1 > 0 then
             z1 = 0;
             z2 = 0;
             else
             z1 = 0;
             z2 = 0;
             end if;
           else
             z1 = 0;
             z2 = 0;
           end if;
     elsewhen zeroCrossing1.y and pre(z2)==1 then
             z1 = 1;
             z2 = 0;
     elsewhen zeroCrossing2.y and pre(z2)==1 then
             z1 = 0;
             z2 = 0;
      end when;
      annotation (Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"1\"><tr>
<td><p>Last update</p></td>
<td><p>25/02/2020</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>M.A.A Murad and L. Vanfretti</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mohammed:murad@ucdconnect.ie\">mohammed.murad@ucdconnect.ie</a></p></td>
</tr>
</table>
</html>"));
    end RelayFeedback_filippov;

  end Example2;

  package Example3
    model smib_db
      "single machine infinite bus system with AVR and PSS, PI controller includes an DB"
      parameter Real xd1 = 0.25 "d-axis transient reactance of gen 1+transformer 
                                      reactance";
      parameter Real xd = 1 "d-axis synchronous reactance of gen 2";
      parameter Real M = 8 "Mechanical starting time of gen 1";
      parameter Real D = 0 "Damping coefficient";
      parameter Real pm = 1 "Mechanical power input from gen 1";
      parameter Real pg0 = 1 "Electrical power input from gen 1";
      parameter Real qg0 = 0.28527 "reactive power input from gen1";
      parameter Real Td1 = 6 "D axis transient time constant gen 1";
      parameter Real x13 = 0.3 "Line reactance 1-3";
      parameter Real x23 = 0.5 "Line reactance 2-3";
      parameter Real v20 = 1;
      parameter Real th20 = 0;
      parameter Real v30 = 0.9623;
      parameter Real vf0 = 1.47888;
      parameter Real kp = 5.5;
      parameter Real ki = 35;
      parameter Real Ta = 0.005;
      parameter Real ka = 2;
      parameter Real kpss = 1.5;
      parameter Real T1 = 0.23;
      parameter Real T2 = 0.12;
      parameter Real c20 = kpss * (1 - T1 / T2);
      parameter Real vmax = 1.60;
      parameter Real vmin = -1.58;
      parameter Real db = 0.0001;
      Real vref(start = 1);
      Real delta(start = 0.70278);
      Real w(start = 0);
      Real eq(start = 1.1001);
      Real v1(start = 1);
      Real v3(start = 0.9623);
      Real th1(start = 0.473451);
      Real th3(start = 0.15648);
      Real pe(start = pg0);
      Real qe(start = 0.28527);
      Real vf(start = vf0);
      Real va(start = 0);
      Real xf(start = vf0) "Integrator state";
      Real s1(start = 0);
      Real c1(start = 0);
      Real c2(start = 0);
      Real c3(start = 0);
      Real ql0;
      Real vf1(start = vf0);
      Real pl0;
      Real xst(start = 17.5);
      Real ytrig1;
      Real ytrig2;

    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing(enable = true) annotation (
        Placement(transformation(extent = {{4, -4}, {24, 16}})));
    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing1(enable = true) annotation (
        Placement(transformation(extent = {{2, -52}, {22, -32}})));
    equation
      der(delta) = w;
      der(w) = 1 / M * (pm - pe);
      der(eq) = 1 / Td1 * (vf1 - xd / xd1 * eq + (xd - xd1) / xd1 * v1 * cos(delta - th1));
      pe = eq * v1 * sin(delta - th1) * (1 / xd1);
      qe = -(v1 ^ 2 - v1 * eq * cos(th1 - delta)) / xd1;
      0 = (-pe) + v3 * v1 * sin(th1 - th3) / x13;
      0 = (-qe) + (v1 ^ 2 - v3 * v1 * cos(th1 - th3)) / x13;
      0 = v1 * v3 * sin(th3 - th1) / x13 + v3 * v20 * sin(th3 - th20) / x23 + pl0 * (v3 / v30);
      0 = (v3 ^ 2 - v3 * v1 * cos(th3 - th1)) / x13 + (v3 ^ 2 - v3 * v20 * cos(th3 - th20)) / x23 + ql0 * (v3 / v30) ^ 2;
      vf = kp * va + xf;
      der(va) = (ka * (vref + c3 - v1) - va) / Ta;
      der(s1) = 1 / T2 * (c2 - s1);
      0 = c1 - w * kpss;
      0 = c2 - c1 * (1 - T1 / T2);
      0 = c3 - c1 * (T1 / T2) - s1;
      if vf >= vmax then
        vf1 = vmax;
      elseif vf <= vmin then
        vf1 = vmin;
      else
        vf1 = vf;
      end if;
      der(xf) = va * (xst + ki - 17.5);
      ytrig1 = vf - (vmax + db / ki * (xst - 17.5));
      ytrig2 = vf - (vmin - db / ki * (xst - 17.5));
      zeroCrossing.u = ytrig1;
      zeroCrossing1.u = ytrig2;
      der(xst) = 0;
      when zeroCrossing.y and der(xf) >= 0 then
        reinit(xst, -pre(xst));
      elsewhen zeroCrossing1.y and der(xf) <= 0 then
        reinit(xst, -pre(xst));
      end when;
      if time > 5 then
        vref = 1.01;
        ql0 = 0.016;
        pl0 = 0.71;
      else
        vref = 1;
        ql0 = 0.01;
        pl0 = 0.7;
      end if;
    end smib_db;

    model smib_ieee
      "single machine infinite bus system with AVR and PSS, PI same as in IEEE standard"
      parameter Real xd1 = 0.25 "d-axis transient reactance of gen 1+transformer 
                                      reactance";
      parameter Real xd = 1 "d-axis synchronous reactance of gen 2";
      parameter Real M = 8 "Mechanical starting time of gen 1";
      parameter Real D = 0 "Damping coefficient";
      parameter Real pm = 1 "Mechanical power input from gen 1";
      parameter Real pg0 = 1 "Electrical power input from gen 1";
      parameter Real qg0 = 0.28527 "reactive power input from gen1";
      parameter Real Td1 = 6 "D axis transient time constant gen 1";
      parameter Real x13 = 0.3 "Line reactance 1-3";
      parameter Real x23 = 0.5 "Line reactance 2-3";
      parameter Real v20 = 1;
      parameter Real th20 = 0;
      parameter Real v30 = 0.9623;
      parameter Real vf0 = 1.47888;
      parameter Real kp = 5.5;
      parameter Real ki = 35;
      parameter Real Ta = 0.005;
      parameter Real ka = 2;
      parameter Real kpss = 1.5;
      parameter Real T1 = 0.23;
      parameter Real T2 = 0.12;
      parameter Real c20 = kpss * (1 - T1 / T2);
      parameter Real vmax = 1.6;
      parameter Real vmin = -1.58;
      parameter Real db = 0.006;
      Real vref(start = 1);
      Real delta(start = 0.70278);
      Real w(start = 0);
      Real eq(start = 1.1001);
      Real v1(start = 1);
      Real v3(start = 0.9623);
      Real th1(start = 0.473451);
      Real th3(start = 0.15648);
      Real pe(start = pg0);
      Real qe(start = 0.28527);
      Real vf(start = vf0);
      Real va(start = 0);
      Real xf(start = vf0)
                          "Integrator state";
      Real s1(start = 0);
      Real c1(start = 0);
      Real c2(start = 0);
      Real c3(start = 0);
      Real ql0;
      Real vf1(start = vf0);
      Real pl0;
    equation
      der(delta) = w;
      der(w) = 1 / M * (pm - pe);
      der(eq) = 1 / Td1 * (vf1 - xd / xd1 * eq + (xd - xd1) / xd1 * v1 * cos(delta - th1));
      pe = eq * v1 * sin(delta - th1) * (1 / xd1);
      qe = -(v1 ^ 2 - v1 * eq * cos(th1 - delta)) / xd1;
      0 = (-pe) + v3 * v1 * sin(th1 - th3) / x13;
      0 = (-qe) + (v1 ^ 2 - v3 * v1 * cos(th1 - th3)) / x13;
      0 = v1 * v3 * sin(th3 - th1) / x13 + v3 * v20 * sin(th3 - th20) / x23 + pl0 * (v3 / v30);
      0 = (v3 ^ 2 - v3 * v1 * cos(th3 - th1)) / x13 + (v3 ^ 2 - v3 * v20 * cos(th3 - th20)) / x23 + ql0 * (v3 / v30) ^ 2;
      vf = kp * va + xf;
      der(va) = (ka * (vref + c3 - v1) - va) / Ta;
      der(s1) = 1 / T2 * (c2 - s1);
      0 = c1 - w * kpss;
      0 = c2 - c1 * (1 - T1 / T2);
      0 = c3 - c1 * (T1 / T2) - s1;
      if vf >= vmax then
        vf1 = vmax;
        der(xf) = 0;
      elseif vf <= vmin then
        vf1 = vmin;
        der(xf) = 0;
      else
        vf1 = vf;
        der(xf) = ki * va;
      end if;
      if time > 5 then
        vref = 1.01;
        ql0 = 0.016;
        pl0 = 0.71;
      else
        vref = 1;
        ql0 = 0.01;
        pl0 = 0.7;
      end if;
    end smib_ieee;

    model smib_filippov
      "single machine infinite bus system with AVR and PSS, considering Filippov method"
      parameter Real xd1 = 0.25 "d-axis transient reactance of gen 1+transformer 
                                      reactance";
      parameter Real xd = 1 "d-axis synchronous reactance of gen 2";
      parameter Real M = 8 "Mechanical starting time of gen 1";
      parameter Real D = 0 "Damping coefficient";
      parameter Real pm = 1 "Mechanical power input from gen 1";
      parameter Real pg0 = 1 "Electrical power input from gen 1";
      parameter Real qg0 = 0.28527 "reactive power input from gen1";
      parameter Real Td1 = 6 "D axis transient time constant gen 1";
      parameter Real x13 = 0.3 "Line reactance 1-3";
      parameter Real x23 = 0.5 "Line reactance 2-3";
      parameter Real v20 = 1;
      parameter Real th20 = 0;
      parameter Real v30 = 0.9623;
      parameter Real vf0 = 1.47888;
      parameter Real kp = 5.5;
      parameter Real ki = 35;
      parameter Real Ta = 0.005;
      parameter Real ka = 2;
      parameter Real kpss = 1.5;
      parameter Real T1 = 0.23;
      parameter Real T2 = 0.12;
      parameter Real c20 = kpss * (1 - T1 / T2);
      parameter Real vmax = 1.60;
      parameter Real vmin = -1.58;
      Real vref(start = 1);
      Real delta(start = 0.70278);
      Real w(start = 0);
      Real eq(start = 1.1001);
      Real v1(start = 1);
      Real v3(start = 0.9623);
      Real th1(start = 0.473451);
      Real th3(start = 0.15648);
      Real pe(start = pg0);
      Real qe(start = 0.28527);
      Real vf(start = vf0);
      Real va(start = 0);
      Real xf(start = vf0)
                          "Integrator state";
      Real s1(start = 0);
      Real c1(start = 0);
      Real c2(start = 0);
      Real c3(start = 0);
      Real ql0;
      Real vf1(start = vf0);
      Real pl0;
      Real a;
      Integer z1(start = 1);
      Integer z2(start = 0);
      Real r1;
      Real f2;
    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing(enable = true) annotation (
        Placement(transformation(extent = {{4, -4}, {24, 16}})));
    public
      Modelica.Blocks.Logical.ZeroCrossing zeroCrossing1(enable = true) annotation (
        Placement(transformation(extent = {{2, -52}, {22, -32}})));
    equation
      der(delta) = w;
      der(w) = 1 / M * (pm - pe);
      der(eq) = 1 / Td1 * (vf1 - xd / xd1 * eq + (xd - xd1) / xd1 * v1 * cos(delta - th1));
      pe = eq * v1 * sin(delta - th1) * (1 / xd1);
      qe = -(v1 ^ 2 - v1 * eq * cos(th1 - delta)) / xd1;
      0 = (-pe) + v3 * v1 * sin(th1 - th3) / x13;
      0 = (-qe) + (v1 ^ 2 - v3 * v1 * cos(th1 - th3)) / x13;
      0 = v1 * v3 * sin(th3 - th1) / x13 + v3 * v20 * sin(th3 - th20) / x23 + pl0 * (v3 / v30);
      0 = (v3 ^ 2 - v3 * v1 * cos(th3 - th1)) / x13 + (v3 ^ 2 - v3 * v20 * cos(th3 - th20)) / x23 + ql0 * (v3 / v30) ^ 2;
      vf = kp * va + xf;
      der(va) = (ka * (vref + c3 - v1) - va) / Ta;
      der(s1) = 1 / T2 * (c2 - s1);
      0 = c1 - w * kpss;
      0 = c2 - c1 * (1 - T1 / T2);
      0 = c3 - c1 * (T1 / T2) - s1;
      if vf >= vmax then
        vf1 = vmax;
      elseif vf <= vmin then
        vf1 = vmin;
      else
        vf1 = vf;
      end if;
     zeroCrossing.u = vf - vmax;
     zeroCrossing1.u = r1;
      a = (ka * (vref + c3 - v1) - va) / Ta;
      der(xf) = ki * va * z1 + kp* a * z2;
      when zeroCrossing.y then
        if r1 * f2 < 0 then
          if r1 > 0 and f2 < 0 then
            z1 = 0;
            z2 = -1;
          else
            z1 = 0;
            z2 = 0;
          end if;
        else
          if r1 > 0 then
            z1 = 0;
            z2 = 0;
          else
            z1 = 1;
            z2 = 0;
          end if;
        end if;
      elsewhen zeroCrossing1.y then
        if pre(z2)==-1 then
          z2 = 0;
          z1 = 1;
        else
          z2 = 0;
          z1 = 1;
        end if;
      end when;
      r1 = kp*a+ki*va;
      f2 = kp*a;
      if time > 5 then
        vref = 1.01;
        ql0 = 0.016;
        pl0 = 0.71;
      else
        vref = 1;
        ql0 = 0.01;
        pl0 = 0.7;
      end if;
    end smib_filippov;

  end Example3;

  annotation (uses(Modelica(version="3.2.2")), Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"1\"><tr>
<td><p>Last update</p></td>
<td><p>25/02/2020</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>M.A.A Murad and L. Vanfretti</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mohammed:murad@ucdconnect.ie\">mohammed.murad@ucdconnect.ie</a></p></td>
</tr>
</table>
</html>"));
end FilippovExamples;

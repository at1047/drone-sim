  % inline Vec3f GetDesAcceleration(Vec3f estPos, Vec3f estVel, Vec3f desPos,
  %                                 Vec3f desVel = Vec3f(0, 0, 0), Vec3f desAcc =
  %                                     Vec3f(0, 0, 0), bool useInt = false) const {
  %   Vec3f posError = desPos - estPos;
  % 
  %   if (useInt) {
  % 
  %     assert(_loopTime > 0);
  %     // Add to accumulated error
  %     _intError = _intError + posError * _loopTime;
  % 
  %     // Apply anti-windup (limit the integrator)
  %     for (int i = 0; i < 3; i++) {
  %       if (_intError[i] > _intLimit) _intError[i] = _intLimit;
  %       if (_intError[i] < -_intLimit) _intError[i] = -_intLimit;
  %     }
  %   }
  % 
  %   return posError * _natFreq * _natFreq
  %       + (desVel - estVel) * 2 * _natFreq * _dampingRatio 
  %       + _intError * _intGain
  %       + desAcc;
  % }

function cmdAcc = GetDesAcceleration(params, estPos, estVel, desPos, desVel, desAcc)

    natFreq = params.natFreq;
    dampingRatio = params.dampingRatio;
    
    posError = desPos - estPos;

    cmdAcc = posError * natFreq^2 + ...
    (desVel - estVel) * 2 * natFreq * dampingRatio + desAcc;

end

function [outCmdAngVel, outCmdThrust] = QuadcopterController(curPos, curVel, curAtt, desPos, desVel, desAcc, ...
                              desiredYawAngle, dt)

end
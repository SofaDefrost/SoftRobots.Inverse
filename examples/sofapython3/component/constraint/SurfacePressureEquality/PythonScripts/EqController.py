#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import numpy as np

_runAsPythonScript = False

def get1DIdx(RowIdx,ColIdx):
    Idx1D = RowIdx * 8 + ColIdx
    return Idx1D

class Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.RootNode = kwargs['RootNode']

        self.Counter = 0
        self.IterationCounter = 0
        self.DistributionStride = 5
        self.begun = False

        self.ModelNode = self.RootNode.model
        self.SurfacePressureEquality = self.ModelNode.AccordeonCavity.SurfacePressureEquality

        self.VolumeChange = 0
        self.VolumeIncrement = 30
        self.SideInflationSign = 1

    def onAnimateBeginEvent(self, eventType):
        pass

    def onKeypressedEvent(self, c):
        pass
        key = c['key']

        if (key == "+"):
            self.VolumeChange = self.VolumeChange + self.VolumeIncrement # a negative volume changes means increasing volume!
            #MeasuredVolumeChanges = [self.VolumeChange, self.VolumeChange, -self.VolumeChange, -self.VolumeChange]

        if (key == "-"):
            self.VolumeChange = self.VolumeChange - self.VolumeIncrement # a negative volume changes means increasing volume!
            #MeasuredVolumeChanges = [self.VolumeChange, self.VolumeChange, -self.VolumeChange, -self.VolumeChange]

        self.setDesiredVolumeEquality(self.VolumeChange)


    def setDesiredVolumeEquality(self, Volume):
        self.SurfacePressureEquality.getData('eqVolumeGrowth').value = Volume

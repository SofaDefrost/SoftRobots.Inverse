#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import numpy as np

_runAsPythonScript = False

class Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.RootNode = kwargs['RootNode']
        self.Counter = 0
        self.IterationCounter = 0
        self.DistributionStride = 5
        self.begun = False

        self.ModelNode = self.RootNode.model

        self.CableEquality = self.ModelNode.Cables.Cable1.CableEquality

        self.CableLengthChange = 0
        self.LengthIncrement = 1
        self.SideInflationSign = 1

    def onAnimateBeginEvent(self, eventType):
        pass

    def onKeypressedEvent(self, c):
        pass
        key = c['key']

        if (key == "+"):
            self.CableLengthChange = self.CableLengthChange + self.LengthIncrement

        if (key == "-"):
            self.CableLengthChange = self.CableLengthChange - self.LengthIncrement

        self.setDesiredLengthEquality(self.CableLengthChange)


    def setDesiredLengthEquality(self, Length):
        self.CableEquality.getData('eqDisp').value = Length

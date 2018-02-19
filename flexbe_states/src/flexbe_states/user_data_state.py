#!/usr/bin/env python

from flexbe_core import EventState, Logger

'''
Created on 14-Feb-2018

@author: David Conner
'''

class UserDataState(EventState):
    '''
    Implements a state that defines user data

    -- data    type        Data for given user data

    #> data                User data

    <= done                Created the user data
    <= param_error         Configuration error - no data set
    '''


    def __init__(self, data ):
        '''
        Constructor
        '''
        super(UserDataState, self).__init__( output_keys=["data"], outcomes=["done","param_err"])

        self._my_data = data
        self._return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        return self._return_code


    def on_enter(self, userdata):

        try:
          # Add the user data
          userdata.data = self._my_data
          self._return_code = 'done'
        except:
          self._return_code = 'param_error'

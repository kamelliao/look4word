#! /usr/bin/env python
import openai
from typing import List, Union, Tuple

openai.api_key = "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"

class Oracle:
    '''
    Perform a variety of decision making tasks with GPT3.
    '''
    def __init__(self):
        self.post = lambda prompt: openai.Completion.create(
            model="text-davinci-003",
            prompt=prompt,
            temperature=0,
            max_tokens=100,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0
        )

    def get_room(self, user_utterance: str) -> str:
        '''
        Parameters
        ----------
        user_utterance : str
            User's command on which room he/she is going.

        Returns
        --------
        str
            The room number indicated in user's utterance.

        Examples
        --------
        >>> oracle.get_room("Lead me to 209.")
        '209'
        '''
        prompt = f"""
        The user said "Take me to room 413.". What is the target room number?
        413

        The user said f"{user_utterance}". What is the target room number?
        """
        response = self.post(prompt.strip())
        return response.choices[0]['text'].strip()

    def elevator_in(self):
        # TODO:
        raise NotImplementedError()

    def elevator_out(self, target: Union[int, str], elevator_prompt: str) -> str:
        '''
        Determine if the user has arrived at the target floor.

        Parameters
        ----------
        target : Union[int, str]
            Target floor number.
        elevator_prompt : str
            Notification of the arrived floor from the elevator. This would be from the output of
            the speech recognition module.
        
        Returns
        --------
        str
            'yes' | 'no', which indicates whether the user should get off the elevator.
        
        Examples
        --------
        >>> oracle.elevator_out(9, "九樓到了")
        'yes'
        >>> oracle.elevator_out(9, "十九樓到了")
        'no'
        '''
        prompt = f"""
        Now I am in the elevator. Given target floor and current floor, tell me whether I should leave the elevator.

        I am going to the 5th floor. Now I hear "八樓到了". Should I go out?
        No

        I am going to the 5th floor. Now I hear "五樓到了". Should I go out?
        Yes

        I am going to the {target}th floor. Now I hear "{elevator_prompt}". Should I go out?
        """

        response = self.post(prompt.strip())
        return response.choices[0]['text'].strip()

    def check_go_forward(self, target: Union[int, str], history: List[Union[int, str]]) -> Tuple[str, str]:
        '''
        Determine whether the user is in the right direction to the target room.

        Parameters
        ----------
        target : Union[int, str]
            Target room number.
        history : List[Union[int, str]]
            Most recently observed room numbers.

        Returns
        --------
        str
            'yes' | 'no', which indicates whether the user is in the right direction.
        str
            'go back' | 'go upstairs' | 'go downstairs', suggest an action if the user is in the wrong direction.

        Examples
        --------
        >>> oracle.check_go_forward(410, [405, 406])
        ('yes', 'continue in the same direction')
        >>> oracle.check_go_forward(410, [213, 214])
        ('no', 'go upstairs')
        >>> oracle.check_go_forward(410, [603, 605])
        ('no', 'go downstairs')
        >>> oracle.check_go_forward(410, [417, 418])
        ('no', 'go back')
        '''
        prompt = f"""
I am in a building with ascending room number. The first digit of room number indicates the floor. Please determine for me whether I should move forward by first check if the floor matched then check if I am in the right direction.

Target: 412. Observed: 410 and 411. 
I want to go to the __ th floor. Now I am at the __ th floor. Am I in the right direction?
4,4,yes,continue in the same direction

Target: 412. Observed: 421 and 422. 
I want to go to the __ th floor. Now I am at the __ th floor. Am I in the right direction?
4,4,no,go back

Target: 412. Observed: 301 and 302. 
I want to go to the __ th floor. Now I am at the __ th floor. Am I in the right direction?
4,3,no,go upstairs

Target: 412. Observed: 801 and 802. 
I want to go to the __ th floor. Now I am at the __ th floor. Am I in the right direction?
4,8,no,go downstairs

Target: {target}. Observed: {" and ".join([str(num) for num in history])}. 
I want to go to the __ th floor. Now I am at the __ th floor. Am I in the right direction?
        """

        response = self.post(prompt.strip())
        response = response.choices[0]['text'].strip().split(',')
        return response[2], response[3]
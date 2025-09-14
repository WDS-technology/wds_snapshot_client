# response_handler.py - Handle server responses properly

import re
import logging

class ResponseHandler:
    @staticmethod
    def parse_response(response):
        """
        Parse server response and extract relevant information
        Returns: (success: bool, message: str, file_path: str or None)
        """
        if not response:
            return False, "Empty response", None
            
        # Remove any tagged response format (RES:uuid:)
        clean_response = response
        if response.startswith("RES:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                clean_response = parts[2]
        
        # Check for success
        if clean_response.startswith("Success:"):
            # Extract file path from success message
            # Format: "Success: [output]\nImage created at [path]\n"
            path_match = re.search(r"Image created at (.+?)(?:\n|$)", clean_response)
            file_path = path_match.group(1) if path_match else None
            return True, clean_response, file_path
            
        # Check for errors
        elif clean_response.startswith("Error"):
            return False, clean_response, None
            
        else:
            # Unknown format, treat as info
            return True, clean_response, None
import docker


class DockerMonitor:
    def __init__(self):
        """Initializes docker monitor."""
        self.client = docker.from_env()

    def close(self):
        print('Closing docker monitor')
        self.client.close()

    def get_tags(self):
        """Returns a list of tags downloaded onto this computer."""
        tags = []
        for image in self.client.images.list():
            for tag in image.tags:
                if tag.startswith(self.repository_name):
                    tokens = tag.split(':')
                    tags.append(tokens[1])
        return tags

    @property
    def repository_name(self):
        """Name of acrobat docker repository."""
        return 'martindeegan/acrobat'

    def build_image_name(self, tag):
        """Construct the image name from a tag."""
        return self.repository_name + ':' + tag

    def clean(self):
        """Removes all images from machine."""
        tags = self.get_tags()
        for tag in tags:
            image_name = self.build_image_name(tag)
            try:
                self.client.images.remove(image_name, force=True)
            except Exception as ex:
                print('Cannot remove {}: {}'.format(tag, str(ex)))

    def check_image(self, tag):
        """Check if image exists in the docker registry."""
        image_name = self.build_image_name(tag)
        try:
            self.client.images.get_registry_data(image_name)
            return True
        except Exception as ex:
            print('Image {} does not exist: '.format(image_name), str(ex))
            return False

    def check_image_local(self, tag):
        """Check if image exists in the machine currently."""
        tags = self.get_tags()
        return (tag in tags)

    def pull_image(self, tag):
        """Pulls the image from the given tag."""
        image_name = self.build_image_name(tag)
        image = self.client.images.pull(image_name)
        return image

    def pull_image_if_not_exists(self, tag):
        """Check if image has been pulled and pull if not. Returns an Image object."""
        image_name = self.build_image_name(tag)
        try:
            # Check if exists
            image = self.client.images.get(image_name)
        except Exception as ex:
            print('Image does not exist: {}'.format(str(ex)))
            # Pull if doesn't exist
            print('Pulling image {}'.format(image_name))
            image = self.client.images.pull(image_name)

        return image

    def create_container(self, image=None, tag=None):
        """Launch a container without starting it with a tag or an image object. Returns a container object."""
        if image is not None:
            image_name = image.tags[0]
        elif tag is not None:
            image_name = self.build_image_name(tag)
        else:
            raise ValueError('No image or tag to run')

        return self.client.containers.create(image_name,
                                             privileged=True,
                                             detach=True,
                                             name='acrobat',
                                             volumes={'/dev/acrobat/fc': {'bind': '/dev/acrobat/fc', 'mode': 'rw'},
                                                      '/dev/acrobat/radio': {'bind': '/dev/acrobat/radio', 'mode': 'rw'},
                                                      '/media/acrobat/logs': {'bind': '/logs', 'mode': 'rw'}})

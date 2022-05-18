/*
 * Hisilicon Kirin SoCs drm master driver
 *
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * Author:
 *	<cailiwei@hisilicon.com>
 *	<zhengwanchun@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/of_platform.h>
#include <linux/component.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "kirin9xx_drm_drv.h"

static int kirin_drm_kms_cleanup(struct drm_device *dev)
{
	struct kirin_drm_private *priv = dev->dev_private;
	static struct kirin_dc_ops const *dc_ops;

	if (priv->fbdev)
		priv->fbdev = NULL;

	dc_ops = of_device_get_match_data(dev->dev);

	drm_kms_helper_poll_fini(dev);
	dc_ops->cleanup(dev);
	drm_mode_config_cleanup(dev);
	devm_kfree(dev->dev, priv);
	dev->dev_private = NULL;

	return 0;
}

static void kirin_fbdev_output_poll_changed(struct drm_device *dev)
{
	struct kirin_drm_private *priv = dev->dev_private;

	dsi_set_output_client(dev);

	drm_fb_helper_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs kirin_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = kirin_fbdev_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void kirin_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.funcs = &kirin_drm_mode_config_funcs;
}

static int kirin_drm_kms_init(struct drm_device *dev)
{
	struct kirin_drm_private *priv = dev->dev_private;
	static struct kirin_dc_ops const *dc_ops;
	int ret;

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev->dev_private = priv;
	dev_set_drvdata(dev->dev, dev);

	/* dev->mode_config initialization */
	drm_mode_config_init(dev);
	kirin_drm_mode_config_init(dev);

	/* display controller init */
	dc_ops = of_device_get_match_data(dev->dev);
	ret = dc_ops->init(dev);
	if (ret)
		goto err_mode_config_cleanup;

	/* bind and init sub drivers */
	ret = component_bind_all(dev->dev, dev);
	if (ret) {
		DRM_ERROR("failed to bind all component.\n");
		goto err_dc_cleanup;
	}

	/* vblank init */
	ret = drm_vblank_init(dev, dev->mode_config.num_crtc);
	if (ret) {
		DRM_ERROR("failed to initialize vblank.\n");
		goto err_unbind_all;
	}
	/* with irq_enabled = true, we can use the vblank feature. */
	dev->irq_enabled = true;

	/* reset all the states of crtc/plane/encoder/connector */
	drm_mode_config_reset(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

#if 1
	/* force detection after connectors init */
	(void)drm_helper_hpd_irq_event(dev);
#endif

	return 0;

err_unbind_all:
	component_unbind_all(dev->dev, dev);
err_dc_cleanup:
	dc_ops->cleanup(dev);
err_mode_config_cleanup:
	drm_mode_config_cleanup(dev);
	devm_kfree(dev->dev, priv);
	dev->dev_private = NULL;

	return ret;
}

DEFINE_DRM_GEM_CMA_FOPS(kirin_drm_fops);

static int kirin_gem_cma_dumb_create(struct drm_file *file,
				     struct drm_device *dev,
				     struct drm_mode_create_dumb *args)
{
	return drm_gem_cma_dumb_create_internal(file, dev, args);
}

static int kirin_drm_connectors_register(struct drm_device *dev)
{
	struct drm_connector_list_iter conn_iter;
	struct drm_connector *failed_connector;
	struct drm_connector *connector;
	int ret;

	mutex_lock(&dev->mode_config.mutex);
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		ret = drm_connector_register(connector);
		if (ret) {
			failed_connector = connector;
			goto err;
		}
	}
	mutex_unlock(&dev->mode_config.mutex);

	return 0;

err:
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (failed_connector == connector)
			break;
		drm_connector_unregister(connector);
	}
	mutex_unlock(&dev->mode_config.mutex);

	return ret;
}

static struct drm_driver kirin_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET |
				  DRIVER_ATOMIC | DRIVER_RENDER,
	.fops				= &kirin_drm_fops,

	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= kirin_gem_cma_dumb_create,

	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	.name			= "kirin9xx",
	.desc			= "Hisilicon Kirin9xx SoCs' DRM Driver",
	.date			= "20170309",
	.major			= 1,
	.minor			= 0,
};

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int kirin_drm_bind(struct device *dev)
{
	struct drm_driver *driver = &kirin_drm_driver;
	struct drm_device *drm_dev;
	struct kirin_drm_private *priv;
	int ret;

	drm_dev = drm_dev_alloc(driver, dev);
	if (!drm_dev)
		return -ENOMEM;

	ret = kirin_drm_kms_init(drm_dev);
	if (ret)
		goto err_drm_dev_unref;

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto err_kms_cleanup;

	drm_fbdev_generic_setup(drm_dev, 32);
	priv = drm_dev->dev_private;

	/* connectors should be registered after drm device register */
	ret = kirin_drm_connectors_register(drm_dev);
	if (ret)
		goto err_drm_dev_unregister;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		 driver->name, driver->major, driver->minor, driver->patchlevel,
		 driver->date, drm_dev->primary->index);

	return 0;

err_drm_dev_unregister:
	drm_dev_unregister(drm_dev);
err_kms_cleanup:
	kirin_drm_kms_cleanup(drm_dev);
err_drm_dev_unref:
	drm_dev_put(drm_dev);

	return ret;
}

static void kirin_drm_unbind(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_dev_unregister(drm_dev);
	kirin_drm_kms_cleanup(drm_dev);
	drm_dev_put(drm_dev);
}

static const struct component_master_ops kirin_drm_ops = {
	.bind = kirin_drm_bind,
	.unbind = kirin_drm_unbind,
};

static int kirin_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct component_match *match = NULL;
	struct device_node *remote;
	static struct kirin_dc_ops const *dc_ops;
	int ret;

	dc_ops = of_device_get_match_data(dev);
	if (!dc_ops) {
		DRM_ERROR("failed to get dt id data\n");
		return -EINVAL;
	}

	DRM_INFO("the device node is %s\n", np->name);
	remote = of_graph_get_remote_node(np, 0, 0);
	if (!remote)
		return -ENODEV;

	DRM_INFO("the device remote node is %s\n", remote->name);

	drm_of_component_match_add(dev, &match, compare_of, remote);
	of_node_put(remote);

	if (ret)
		DRM_ERROR("cma device init failed!");
	return component_master_add_with_match(dev, &kirin_drm_ops, match);
}

static int kirin_drm_platform_remove(struct platform_device *pdev)
{
	static struct kirin_dc_ops const *dc_ops;

	dc_ops = of_device_get_match_data(&pdev->dev);
	component_master_del(&pdev->dev, &kirin_drm_ops);
	return 0;
}

static int kirin_drm_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
	static struct kirin_dc_ops const *dc_ops;
	struct device *dev = &pdev->dev;

	dc_ops = of_device_get_match_data(dev);

	DRM_INFO("+. pdev->name is %s, m_message is %d \n", pdev->name, state.event);
	if (!dc_ops) {
		DRM_ERROR("dc_ops is NULL\n");
		return -EINVAL;
	}
	dc_ops->suspend(pdev, state);

	return 0;
}

static int kirin_drm_platform_resume(struct platform_device *pdev)
{
	static struct kirin_dc_ops const *dc_ops;
	struct device *dev = &pdev->dev;

	dc_ops = of_device_get_match_data(dev);

	if (!dc_ops) {
		DRM_ERROR("dc_ops is NULL\n");
		return -EINVAL;
	}
	dc_ops->resume(pdev);

	return 0;
}

static const struct of_device_id kirin_drm_dt_ids[] = {
	{ .compatible = "hisilicon,kirin960-dpe",
	  .data = &dss_dc_ops,
	},
	{ .compatible = "hisilicon,kirin970-dpe",
	  .data = &dss_dc_ops,
	},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, kirin_drm_dt_ids);

static struct platform_driver kirin_drm_platform_driver = {
	.probe = kirin_drm_platform_probe,
	.remove = kirin_drm_platform_remove,
	.suspend = kirin_drm_platform_suspend,
	.resume = kirin_drm_platform_resume,
	.driver = {
		.name = "kirin9xx-drm",
		.of_match_table = kirin_drm_dt_ids,
	},
};

module_platform_driver(kirin_drm_platform_driver);

MODULE_AUTHOR("cailiwei <cailiwei@hisilicon.com>");
MODULE_AUTHOR("zhengwanchun <zhengwanchun@hisilicon.com>");
MODULE_DESCRIPTION("hisilicon Kirin SoCs' DRM master driver");
MODULE_LICENSE("GPL v2");
